"""Inspect retained MTC introspection and model parameters; no robot actions."""
import json
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from moveit_task_constructor_msgs.msg import TaskDescription, TaskStatistics, Solution
from moveit_task_constructor_msgs.srv import GetSolution
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('codex_readonly_task_diagnostic')
result = {'time_unix': time.time(), 'retained': {}}
qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
subscriptions = []
for name, kind in [('/description', TaskDescription), ('/statistics', TaskStatistics),
                   ('/solution', Solution)]:
    def receive(msg, name=name):
        result['retained'].setdefault(name, []).append(message_to_ordereddict(msg))
    subscriptions.append(node.create_subscription(kind, name, receive, qos))

def call(kind, name, request):
    client = node.create_client(kind, name)
    if not client.wait_for_service(timeout_sec=4.0):
        return {'error': 'unavailable'}
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    return message_to_ordereddict(future.result()) if future.done() else {'error': 'timeout'}

names = ['robot_description', 'robot_description_semantic', 'tool', 'ik_frame',
         'robot_description_kinematics.ur_arm.kinematics_solver',
         'robot_description_kinematics.ur_arm.kinematics_solver_timeout']
for service in ['/mtc_test_task_node/get_parameters', '/move_group/get_parameters',
                '/robot_state_publisher/get_parameters']:
    query_names = ['robot_description'] if 'robot_state_publisher' in service else names
    request = GetParameters.Request(names=query_names)
    result[service] = {'names': query_names, 'response': call(GetParameters, service, request)}
deadline = time.monotonic() + 4.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
result['message_fields'] = {
    'GetSolution.Request': GetSolution.Request.get_fields_and_field_types(),
    'TaskStatistics': TaskStatistics.get_fields_and_field_types(),
    'Solution': Solution.get_fields_and_field_types()}
print(json.dumps(result, separators=(',', ':')))
node.destroy_node()
rclpy.shutdown()
