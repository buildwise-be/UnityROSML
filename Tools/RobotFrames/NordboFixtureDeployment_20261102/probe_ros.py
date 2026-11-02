"""Passive ROS deployment probe: subscriptions and parameter reads only."""
import hashlib
import json
import math
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import String

rclpy.init()
node = Node('codex_fixture_deployment_readonly_probe')
joints = []
active_goals = {}
execution = {}
subscriptions = []
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)


def status_callback(message, topic):
    active_goals[topic] = [g.status for g in message.status_list if g.status in (1, 2, 3)]


def execution_callback(message, topic):
    try:
        execution[topic] = json.loads(message.data)
    except ValueError:
        execution[topic] = {'raw': message.data[:300]}


subscriptions.append(node.create_subscription(JointState, '/joint_states',
    lambda m: joints.append({'time': time.monotonic(), 'name': list(m.name),
                            'position': list(m.position), 'velocity': list(m.velocity)}), qos_profile_sensor_data))
until = time.monotonic() + 1
while time.monotonic() < until:
    rclpy.spin_once(node, timeout_sec=.1)
topics = dict(node.get_topic_names_and_types())
for topic, types in topics.items():
    if 'action_msgs/msg/GoalStatusArray' in types:
        subscriptions.append(node.create_subscription(GoalStatusArray, topic,
            lambda m, t=topic: status_callback(m, t), qos))
for topic in ('/mtc/execution_status', '/mtc/tcp_execution_status', '/mtc/fixture/status', '/commander/motion_status'):
    if topic in topics:
        subscriptions.append(node.create_subscription(String, topic,
            lambda m, t=topic: execution_callback(m, t), 10))
until = time.monotonic() + 2.5
while time.monotonic() < until:
    rclpy.spin_once(node, timeout_sec=.05)
models = {}


def parameter_call(service, name, request):
    client = node.create_client(service, name)
    try:
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError(name + ' unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done():
            future.cancel()
            raise RuntimeError(name + ' timed out')
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()
    finally:
        node.destroy_client(client)


def normalized_model_signature(robot, exclude_safety_controllers=False):
    """Hash root link/joint structure; optionally separate driver-only soft limits."""
    def value(text):
        tokens = (text or '').split()
        try:
            numbers = [float(token) for token in tokens]
            if tokens and all(math.isfinite(number) for number in numbers):
                return ' '.join('0' if number == 0 else format(number, '.15g') for number in numbers)
        except ValueError:
            pass
        return ' '.join(tokens)

    def element(item):
        return [item.tag, sorted((key, value(content)) for key, content in item.attrib.items()),
                value(item.text), [element(child) for child in item
                    if not (exclude_safety_controllers and item.tag == 'joint' and child.tag == 'safety_controller')]]

    selected = sorted((item for item in robot if item.tag in ('link', 'joint')),
                      key=lambda item: (item.tag, item.get('name', '')))
    if not any(item.tag == 'link' for item in selected) or not any(item.tag == 'joint' for item in selected):
        raise ValueError('robot_description has no complete root link/joint model')
    serialized = json.dumps([element(item) for item in selected], separators=(',', ':'), ensure_ascii=True)
    return hashlib.sha256(serialized.encode()).hexdigest()


for name in ('/robot_state_publisher', '/move_group', '/mtc_test_task_node', '/controller_manager'):
    description_expected = name != '/controller_manager'
    record = {'description_expected': description_expected, 'description_sha256': None,
              'description_length': None, 'robot_link_joint_sha256': None, 'tool': None,
              'full_robot_link_joint_sha256': None, 'safety_controller_metadata': None,
              'fixture_enabled': None, 'fixture_parameter_present': False,
              'fixture_calibration_output': None, 'mount_height_m': None, 'tcp_local_xyz': None,
              'expected_mount_height_m': .05525, 'mount_height_matches_expected': None}
    try:
        listed = parameter_call(ListParameters, name + '/list_parameters', ListParameters.Request())
        declared = set(listed.result.names)
        wanted = ['robot_description', 'tool', 'fixture.enabled', 'fixture.calibration_output']
        selected = [key for key in wanted if key in declared]
        record['queried_parameters'] = selected
        record['undeclared_parameters'] = [key for key in wanted if key not in declared]
        values = {}
        if selected:
            request = GetParameters.Request()
            request.names = selected
            response = parameter_call(GetParameters, name + '/get_parameters', request)
            if len(response.values) != len(selected):
                raise RuntimeError('GetParameters returned %d values for %d declared names' %
                                   (len(response.values), len(selected)))
            values = dict(zip(selected, response.values))

        def typed(key, expected_type, field):
            parameter = values.get(key)
            if parameter is None:
                return None
            if parameter.type != expected_type:
                raise ValueError('%s has type %d, expected %d' % (key, parameter.type, expected_type))
            return getattr(parameter, field)

        description = typed('robot_description', ParameterType.PARAMETER_STRING, 'string_value')
        record['tool'] = typed('tool', ParameterType.PARAMETER_STRING, 'string_value')
        record['fixture_parameter_present'] = 'fixture.enabled' in values
        record['fixture_enabled'] = typed('fixture.enabled', ParameterType.PARAMETER_BOOL, 'bool_value')
        record['fixture_calibration_output'] = typed('fixture.calibration_output', ParameterType.PARAMETER_STRING, 'string_value')
        if description:
            record['description_sha256'] = hashlib.sha256(description.encode()).hexdigest()
            record['description_length'] = len(description)
            robot = ET.fromstring(description)
            if robot.tag != 'robot':
                raise ValueError('robot_description root is not robot')
            record['full_robot_link_joint_sha256'] = normalized_model_signature(robot)
            record['robot_link_joint_sha256'] = normalized_model_signature(robot, exclude_safety_controllers=True)
            safety = {}
            for arm_joint in robot.findall('joint'):
                controllers = []
                for controller in arm_joint.findall('safety_controller'):
                    attributes = {key: float(value) for key, value in sorted(controller.attrib.items())}
                    if not all(math.isfinite(value) for value in attributes.values()):
                        raise ValueError('Nonfinite safety_controller metadata')
                    controllers.append(attributes)
                if controllers:
                    safety[arm_joint.get('name')] = controllers
            record['safety_controller_metadata'] = safety
            record['link_count'] = len(robot.findall('link'))
            record['joint_count'] = len(robot.findall('joint'))
            joint = robot.find("joint[@name='kenos_sl_tool_joint']/origin")
            if joint is not None:
                xyz = [float(token) for token in joint.get('xyz', '').split()]
                if len(xyz) != 3 or not all(math.isfinite(number) for number in xyz):
                    raise ValueError('Kenos SL mount origin is malformed')
                record['mount_height_m'] = xyz[2]
                record['mount_height_matches_expected'] = abs(xyz[2] - .05525) <= 1e-9
            tcp = robot.find("joint[@name='kenos_sl_tcp_fixed']/origin")
            record['tcp_local_xyz'] = tcp.get('xyz') if tcp is not None else None
        elif description_expected:
            record['error'] = 'robot_description is undeclared or empty'
        else:
            record['description_note'] = 'controller_manager may receive robot_description by topic'
        if name == '/mtc_test_task_node' and 'tool' not in values:
            record['error'] = 'MTC startup tool parameter is undeclared'
    except Exception as error:
        record['error'] = '%s: %s' % (type(error).__name__, error)
    models[name] = record

# Parameter reads may outlast the first observation window. Refresh passive
# joint/action feedback before deciding whether the arm is stationary now.
until = time.monotonic() + 2.5
while time.monotonic() < until:
    rclpy.spin_once(node, timeout_sec=.05)
arm = {name for name in (joints[-1]['name'] if joints else []) if name.endswith(('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'))}
recent = [j for j in joints if j['time'] >= time.monotonic() - 5]
complete = [j for j in recent if len(j['name']) == len(set(j['name'])) and
            len(j['position']) == len(j['name']) and len(j['velocity']) == len(j['name']) and
            arm.issubset(j['name']) and all(math.isfinite(value) for value in j['position'] + j['velocity'])]
velocities = [abs(v) for j in complete for n, v in zip(j['name'], j['velocity']) if n in arm]
positions = {n: [j['position'][j['name'].index(n)] for j in complete] for n in arm}
drift = max((max(v)-min(v) for v in positions.values() if v), default=999)
busy_status = any(str(v.get('status', '')).lower() in ('executing', 'running', 'accepted', 'starting') for v in execution.values())
stationary = len(arm) == 6 and len(complete) >= 5 and bool(velocities) and max(velocities) < .005 and drift < .002
model_nodes = ('/robot_state_publisher', '/move_group', '/mtc_test_task_node')
signatures = [models[name].get('robot_link_joint_sha256') for name in model_nodes]
models_match = all(signatures) and len(set(signatures)) == 1 and not any(models[name].get('error') for name in model_nodes)
full_signatures = [models[name].get('full_robot_link_joint_sha256') for name in model_nodes]
safety_by_node = {name: models[name].get('safety_controller_metadata') for name in model_nodes}
safety_reference = safety_by_node[model_nodes[0]] or {}
safety_differences = {name: sorted(joint for joint in set(safety_reference) | set(metadata or {})
                                 if safety_reference.get(joint) != (metadata or {}).get(joint))
                      for name, metadata in safety_by_node.items() if metadata != safety_reference}
report = {'stationary': stationary, 'joint_messages': len(recent), 'joint_names': sorted(arm),
          'complete_joint_messages': len(complete),
          'max_abs_joint_velocity': max(velocities, default=None), 'max_joint_drift': drift,
          'active_goals': active_goals, 'execution_status': execution,
          'idle': stationary and not any(active_goals.values()) and not busy_status,
          'models': models, 'robot_link_joint_models_match': bool(models_match),
          'full_robot_link_joint_models_match': bool(all(full_signatures) and len(set(full_signatures)) == 1),
          'comparisonExcludedElements': ['safety_controller'],
          'model_comparison_scope': 'Complete root link/joint subtrees including hard limits, origins, axes, geometry and inertials; only joint safety_controller children are separated',
          'safety_controller_metadata_by_node': safety_by_node,
          'safety_controller_differences_from_rsp': safety_differences,
          'model_comparison_nodes': list(model_nodes),
          'topics': sorted(topics), 'services': sorted(name for name, _ in node.get_service_names_and_types()),
          'robot_motion_requested': False, 'execution_requests_sent': False,
          'io_commands_sent': False, 'production_plan_requests_sent': False}
print(json.dumps(report))
node.destroy_node()
rclpy.shutdown()
