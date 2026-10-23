"""Read-only ROS diagnostics. Never publishes, plans, executes, or changes scene."""
import json
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('codex_readonly_scene_diagnostic')
result = {'time_unix': time.time(), 'statuses': {}}
subscriptions = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
for topic in ['/mtc/planning_status', '/mtc/tcp_planning_status',
              '/mtc/execution_status', '/mtc/tcp_execution_status']:
    def receive(msg, name=topic):
        try:
            result['statuses'][name] = json.loads(msg.data)
        except Exception:
            result['statuses'][name] = msg.data
    subscriptions.append(node.create_subscription(String, topic, receive, qos))

def call(service_type, name, request):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=5.0):
        result[name + '_error'] = 'service unavailable'
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        result[name + '_error'] = 'query timeout'
        return None
    return future.result()

request = GetPlanningScene.Request()
request.components.components = (
    PlanningSceneComponents.SCENE_SETTINGS |
    PlanningSceneComponents.ROBOT_STATE |
    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
    PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
    PlanningSceneComponents.TRANSFORMS)
response = call(GetPlanningScene, '/get_planning_scene', request)
if response:
    result['scene'] = message_to_ordereddict(response.scene)
    validity_request = GetStateValidity.Request()
    validity_request.robot_state = response.scene.robot_state
    validity_request.group_name = 'ur_arm'
    validity = call(GetStateValidity, '/check_state_validity', validity_request)
    if validity:
        result['current_state_validity'] = message_to_ordereddict(validity)

deadline = time.monotonic() + 3.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
result['topics'] = node.get_topic_names_and_types()
result['services'] = node.get_service_names_and_types()
print(json.dumps(result, separators=(',', ':')))
node.destroy_node()
rclpy.shutdown()
