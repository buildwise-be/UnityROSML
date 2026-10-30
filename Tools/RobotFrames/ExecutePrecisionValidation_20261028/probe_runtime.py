"""Read-only runtime probe. Never publishes a robot command or changes parameters."""
import hashlib
import json
import math
import sys
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers

rclpy.init()
node = rclpy.create_node('codex_mtc_precision_readonly_probe')
out = {'execution': None, 'tcp_execution': None, 'planning': None, 'actions': {}, 'joint_feedback': None}
subscriptions = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
for topic, name in [('/mtc/execution_status', 'execution'), ('/mtc/planning_status', 'planning'),
                    ('/mtc/tcp_execution_status', 'tcp_execution')]:
    def status(msg, key=name):
        data = json.loads(msg.data)
        out[key] = ({'overall_success': data.get('overall_success'), 'message': data.get('message')}
                    if key == 'planning' else data)
    subscriptions.append(node.create_subscription(String, topic, status, qos))

joint_received = None
def joints(msg):
    global joint_received
    joint_received = time.monotonic()
    out['joint_feedback'] = {'names': list(msg.name), 'positions': list(msg.position),
                             'stamp': [msg.header.stamp.sec, msg.header.stamp.nanosec]}
subscriptions.append(node.create_subscription(JointState, '/joint_states', joints,
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))

deadline = time.monotonic() + 3.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
controller_client = node.create_client(ListControllers, '/controller_manager/list_controllers')
assert controller_client.wait_for_service(timeout_sec=3), 'Controller-manager service unavailable'
controller_future = controller_client.call_async(ListControllers.Request())
rclpy.spin_until_future_complete(node, controller_future, timeout_sec=5)
assert controller_future.done() and controller_future.result() is not None
out['controllers'] = [{'name': c.name, 'state': c.state, 'type': c.type}
                      for c in controller_future.result().controller]
offered_actions = []
for topic, types in node.get_topic_names_and_types():
    if 'action_msgs/msg/GoalStatusArray' not in types:
        continue
    publishers = node.get_publishers_info_by_topic(topic)
    if not publishers:
        continue
    offered_actions.append(topic)
    def action_status(msg, key=topic):
        out['actions'][key] = [{'status': int(s.status), 'goal': bytes(s.goal_info.goal_id.uuid).hex()}
                              for s in msg.status_list]
    offered_qos = publishers[0].qos_profile
    # Discovery can report UNKNOWN history/depth; use concrete subscriber values.
    status_qos = QoSProfile(depth=10,
        reliability=(ReliabilityPolicy.BEST_EFFORT if offered_qos.reliability == ReliabilityPolicy.BEST_EFFORT
                     else ReliabilityPolicy.RELIABLE),
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(GoalStatusArray, topic, action_status, status_qos))

client = node.create_client(GetParameters, '/mtc_test_task_node/get_parameters')
assert client.wait_for_service(timeout_sec=4), 'MTC parameter service unavailable'
names = ['robot_description', 'robot_description_semantic', 'tool', 'ik_frame',
         'execution_start_tolerance', 'execution_joint_state_max_age']
request = GetParameters.Request(names=names)
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=5)
assert future.done() and future.result() is not None, 'MTC parameter query timed out'
parameters = {}
for key, value in zip(names, future.result().values):
    if key in ['robot_description', 'robot_description_semantic']:
        assert value.type == 4 and value.string_value
        parameters[key + '_sha256'] = hashlib.sha256(value.string_value.encode()).hexdigest()
    elif value.type == 4:
        parameters[key] = value.string_value
    elif value.type == 3:
        parameters[key] = value.double_value
    else:
        raise RuntimeError('Unexpected parameter type: ' + key)
out['parameters'] = parameters

deadline = time.monotonic() + 3.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
out['action_topics'] = offered_actions
required_actions = [
    '/execute_task_solution/_action/status', '/execute_trajectory/_action/status',
    '/move_action/_action/status',
    '/sequence_move_group/_action/status', '/trajectory_until_node/execute/_action/status',
    '/ur_robot_state_helper/set_mode/_action/status',
]
for controller in out['controllers']:
    if controller['state'] != 'active':
        continue
    if ('JointTrajectoryController' in controller['type'] or
            'PassthroughTrajectoryController' in controller['type']):
        required_actions.append('/' + controller['name'] + '/follow_joint_trajectory/_action/status')
    if controller['name'] == 'tool_contact_controller':
        required_actions.append('/tool_contact_controller/detect_tool_contact/_action/status')
out['required_action_topics'] = required_actions
out['missing_action_topics'] = [t for t in required_actions if t not in offered_actions]
out['missing_action_status'] = [t for t in offered_actions if t not in out['actions']]
out['active_goals'] = [{'topic': t, **s} for t, statuses in out['actions'].items()
                       for s in statuses if s['status'] in [1, 2, 3]]
out['joint_age_seconds'] = time.monotonic() - joint_received if joint_received is not None else None
valid_joints = (out['joint_feedback'] is not None and out['joint_age_seconds'] < 1.0 and
                len(out['joint_feedback']['names']) == len(out['joint_feedback']['positions']) and
                all(math.isfinite(v) for v in out['joint_feedback']['positions']) and
                len([n for n in out['joint_feedback']['names'] if n.startswith('ur_')]) >= 6)
out['valid_fresh_joints'] = valid_joints
idle_statuses = ['ready', 'rejected', 'failed', 'succeeded', 'planning_failed']
out['idle'] = (out['execution'] is not None and
               out['execution'].get('status') in idle_statuses and
               (out['tcp_execution'] is None or out['tcp_execution'].get('status') in idle_statuses + ['invalidated']) and
               not out['active_goals'] and
               not out['missing_action_topics'])
print(json.dumps(out))
node.destroy_node()
rclpy.shutdown()
if '--require-idle' in sys.argv:
    assert out['idle'] and valid_joints, 'Current MTC/action state is not proven idle'
else:
    assert (valid_joints and not out['active_goals'] and
            not out['missing_action_topics'])
    assert out['execution'] is None or out['execution'].get('status') in idle_statuses + ['invalidated']
    assert out['tcp_execution'] is None or out['tcp_execution'].get('status') in idle_statuses + ['invalidated']
