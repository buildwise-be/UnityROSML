"""Query IK/collision services only. No planning, scene changes, IO or execution."""
import copy
import json
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetPositionFK, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
from moveit_task_constructor_msgs.srv import GetSolution
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('codex_readonly_ik_diagnostic')
result = {'time_unix': time.time(), 'ik': [], 'controls': []}
clients = {}

def call(kind, name, request):
    if name not in clients:
        clients[name] = node.create_client(kind, name)
        if not clients[name].wait_for_service(timeout_sec=4.0):
            raise RuntimeError('Unavailable ' + name)
    future = clients[name].call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    if not future.done():
        raise RuntimeError('Timeout ' + name)
    return future.result()

scene_request = GetPlanningScene.Request()
scene_request.components.components = (PlanningSceneComponents.ROBOT_STATE |
    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
scene = call(GetPlanningScene, '/get_planning_scene', scene_request).scene
result['joint_state'] = message_to_ordereddict(scene.robot_state.joint_state)
for name, kinds in node.get_service_names_and_types():
    if kinds == ['moveit_task_constructor_msgs/srv/GetSolution']:
        for solution_id in [1, 2]:
            request = GetSolution.Request(solution_id=solution_id)
            result['solution_' + str(solution_id)] = message_to_ordereddict(
                call(GetSolution, name, request))

def multiply(a, b):
    x,y,z,w = a
    X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)

def query_pose(pose, label):
    item = {'name': label, 'pose': message_to_ordereddict(pose)}
    for avoid in [False, True]:
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur_arm'
        request.ik_request.robot_state = scene.robot_state
        request.ik_request.avoid_collisions = avoid
        request.ik_request.ik_link_name = 'tcp'
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout.sec = 1
        answer = call(GetPositionIK, '/compute_ik', request)
        key = 'collision_checked' if avoid else 'kinematic_only'
        item[key] = {'error': message_to_ordereddict(answer.error_code)}
        if answer.error_code.val == 1:
            item[key]['joint_state'] = message_to_ordereddict(answer.solution.joint_state)
            vr = GetStateValidity.Request(robot_state=answer.solution,group_name='ur_arm')
            item[key]['validity'] = message_to_ordereddict(
                call(GetStateValidity,'/check_state_validity',vr))
    return item

fk_request = GetPositionFK.Request()
fk_request.header.frame_id = 'mir_base_footprint'
fk_request.robot_state = scene.robot_state
fk_request.fk_link_names = ['ur_base_link','ur_shoulder_link','ur_upper_arm_link',
                           'ur_forearm_link','ur_wrist_1_link','ur_wrist_2_link',
                           'ur_wrist_3_link','tcp']
fk_answer = call(GetPositionFK, '/compute_fk', fk_request)
result['fk'] = message_to_ordereddict(fk_answer)
if fk_answer.error_code.val == 1:
    current_tcp = fk_answer.pose_stamped[-1]
    result['controls'].append(query_pose(current_tcp,'current_tcp_positive_control'))

# MTC cylinder markers have their centre half an axis-length away from the frame.
# The blue cylinder retains the frame quaternion and represents its local +Z.
for trajectory in result.get('solution_2',{}).get('solution',{}).get('sub_trajectory',[]):
    markers = trajectory['info']['markers']
    for marker in markers:
        if marker['ns'] != 'target frame' or marker['color']['b'] < 0.7:
            continue
        p = marker['pose']['position']
        q = marker['pose']['orientation']
        axis = (2*(q['x']*q['z']+q['w']*q['y']),
                2*(q['y']*q['z']-q['w']*q['x']),
                1-2*(q['x']*q['x']+q['y']*q['y']))
        pose = PoseStamped()
        pose.header.frame_id = marker['header']['frame_id']
        for component, n in zip(['x','y','z'],axis):
            setattr(pose.pose.position,component,p[component]-marker['scale']['z']/2*n)
        for component in ['x','y','z','w']:
            setattr(pose.pose.orientation,component,q[component])
        result['controls'].append(query_pose(pose,'recovered_failed_tcp_target_frame'))
        for shift in [0.1,0.2,0.3]:
            shifted = copy.deepcopy(pose)
            shifted.pose.position.y -= shift
            result['controls'].append(query_pose(shifted,'tcp_target_y_minus_' + str(shift)))

for obj in scene.world.collision_objects:
    if obj.id != 'object':
        continue
    q = obj.pose.orientation
    base_q = (q.x,q.y,q.z,q.w)
    normal = (2*(q.x*q.z+q.w*q.y), 2*(q.y*q.z-q.w*q.x),
              1-2*(q.x*q.x+q.y*q.y))
    result['object'] = message_to_ordereddict(obj)
    result['object_normal'] = normal
    for variant in ['actual', 'normal_flipped_for_comparison']:
        variant_q = base_q if variant == 'actual' else multiply(base_q,(1,0,0,0))
        variant_normal = normal if variant == 'actual' else tuple(-v for v in normal)
        for i in range(12):
            angle = i*math.pi/6
            target_q = multiply(variant_q,(0,0,math.sin(angle/2),math.cos(angle/2)))
            pose = PoseStamped()
            pose.header.frame_id = obj.header.frame_id
            pose.pose = copy.deepcopy(obj.pose)
            for axis, value in zip(['x','y','z'], variant_normal):
                setattr(pose.pose.position, axis,
                        getattr(obj.pose.position, axis)-0.102*value)
            for axis,value in zip(['x','y','z','w'],target_q):
                setattr(pose.pose.orientation,axis,value)
            item = {'variant': variant, 'yaw_degrees': i*30,
                    'target': message_to_ordereddict(pose)}
            for avoid in [False, True]:
                request = GetPositionIK.Request()
                request.ik_request.group_name = 'ur_arm'
                request.ik_request.robot_state = scene.robot_state
                request.ik_request.avoid_collisions = avoid
                request.ik_request.ik_link_name = 'tcp'
                request.ik_request.pose_stamped = pose
                request.ik_request.timeout.nanosec = 300000000
                answer = call(GetPositionIK, '/compute_ik', request)
                key = 'collision_checked' if avoid else 'kinematic_only'
                item[key] = {'error_code': answer.error_code.val,
                             'error': message_to_ordereddict(answer.error_code)}
                if answer.error_code.val == 1:
                    item[key]['joint_state'] = message_to_ordereddict(answer.solution.joint_state)
                    vr = GetStateValidity.Request(robot_state=answer.solution,group_name='ur_arm')
                    item[key]['validity'] = message_to_ordereddict(
                        call(GetStateValidity,'/check_state_validity',vr))
            result['ik'].append(item)

print(json.dumps(result,separators=(',',':')))
node.destroy_node()
rclpy.shutdown()
