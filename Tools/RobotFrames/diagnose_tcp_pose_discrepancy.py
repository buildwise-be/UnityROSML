"""Read-only comparison of a retained TCP goal, current FK, and measured TCP."""
import copy
import json
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('codex_readonly_tcp_pose_comparison')
result = {'time_unix':time.time(), 'queries':[]}
subscription = node.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose',
    lambda msg:result.update(measured_controller_tcp=message_to_ordereddict(msg)),
    QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
clients = {}
def call(kind,name,request):
    if name not in clients:
        clients[name] = node.create_client(kind,name)
        if not clients[name].wait_for_service(timeout_sec=4):
            raise RuntimeError('Unavailable ' + name)
    future = clients[name].call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=8)
    if not future.done():
        raise RuntimeError('Timeout ' + name)
    return future.result()

sr = GetPlanningScene.Request()
sr.components.components = PlanningSceneComponents.ROBOT_STATE
state = call(GetPlanningScene,'/get_planning_scene',sr).scene.robot_state
fr = GetPositionFK.Request()
fr.header.frame_id = 'mir_base_footprint'
fr.fk_link_names = ['ur_base_link','ur_base','ur_flange','ur_tool0','ur_wrist_3_link','tcp']
fr.robot_state = state
fk = call(GetPositionFK,'/compute_fk',fr)
result['fk'] = message_to_ordereddict(fk)
result['joint_state'] = message_to_ordereddict(state.joint_state)
current = fk.pose_stamped[-1]
goal = copy.deepcopy(current)
goal.pose.position.x,goal.pose.position.y,goal.pose.position.z = (
    0.303459,0.855156,1.268516)
goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w = (
    -0.4440968831461615,0.5845182195607416,0.4788435565704329,0.48148235451819416)
mixed_position = copy.deepcopy(current)
mixed_position.pose.position = copy.deepcopy(goal.pose.position)
mixed_rotation = copy.deepcopy(current)
mixed_rotation.pose.orientation = copy.deepcopy(goal.pose.orientation)
poses = [('current_control',current),('retained_A',goal),
         ('A_position_current_rotation',mixed_position),
         ('current_position_A_rotation',mixed_rotation)]
for axis in ['x','y','z']:
    for offset in [-0.02,0.02]:
        pose = copy.deepcopy(current)
        setattr(pose.pose.position,axis,getattr(pose.pose.position,axis)+offset)
        poses.append(('current_' + axis + ('_plus_2cm' if offset>0 else '_minus_2cm'),pose))
for label,pose in poses:
    item = {'name':label,'pose':message_to_ordereddict(pose)}
    for avoid in [False,True]:
        ir = GetPositionIK.Request()
        ir.ik_request.group_name = 'ur_arm'
        ir.ik_request.ik_link_name = 'tcp'
        ir.ik_request.pose_stamped = pose
        ir.ik_request.robot_state = state
        ir.ik_request.avoid_collisions = avoid
        ir.ik_request.timeout.sec = 1
        answer = call(GetPositionIK,'/compute_ik',ir)
        key = 'collision_checked' if avoid else 'kinematic_only'
        item[key] = {'error':message_to_ordereddict(answer.error_code)}
        if answer.error_code.val == 1:
            vr = GetStateValidity.Request(robot_state=answer.solution,group_name='ur_arm')
            item[key]['validity'] = message_to_ordereddict(
                call(GetStateValidity,'/check_state_validity',vr))
    result['queries'].append(item)
deadline = time.monotonic()+1
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=0.1)
print(json.dumps(result,separators=(',',':')))
node.destroy_node()
rclpy.shutdown()
