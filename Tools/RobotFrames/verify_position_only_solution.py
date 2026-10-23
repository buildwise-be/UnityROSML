"""Verify an offline candidate through read-only ROS FK/IK/collision services."""
import json
import time
import rclpy
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene,GetPositionFK,GetPositionIK,GetStateValidity
from rosidl_runtime_py.convert import message_to_ordereddict as to_dict

rclpy.init()
node=rclpy.create_node('codex_readonly_verify_orientation_solution')
clients={}
def call(kind,name,request):
    if name not in clients:
        clients[name]=node.create_client(kind,name)
        if not clients[name].wait_for_service(timeout_sec=4):
            raise RuntimeError('Unavailable '+name)
    future=clients[name].call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=8)
    if not future.done():
        raise RuntimeError('Timeout '+name)
    return future.result()
sr=GetPlanningScene.Request()
sr.components.components=PlanningSceneComponents.ROBOT_STATE
current=call(GetPlanningScene,'/get_planning_scene',sr).scene.robot_state
candidate=type(current)()
positions={
    'ur_shoulder_pan_joint':0.21368822116518552,
    'ur_shoulder_lift_joint':-2.379521133910478,
    'ur_elbow_joint':-0.0016694315772912837,
    'ur_wrist_1_joint':-0.5331527545272331,
    'ur_wrist_2_joint':1.2973885033086483,
    'ur_wrist_3_joint':-0.2668867126076326}
candidate.joint_state.name=list(positions)
candidate.joint_state.position=list(positions.values())
fr=GetPositionFK.Request()
fr.header.frame_id='mir_base_footprint'
fr.fk_link_names=['tcp']
fr.robot_state=candidate
fk=call(GetPositionFK,'/compute_fk',fr)
vr=GetStateValidity.Request(robot_state=candidate,group_name='ur_arm')
result={'time_unix':time.time(),'candidate_joint_state':to_dict(candidate.joint_state),
        'fk':to_dict(fk),'validity':to_dict(call(GetStateValidity,'/check_state_validity',vr)),
        'ik':[]}
for label,seed in [('current_state',current),('candidate_state',candidate)]:
    for avoid in [False,True]:
        ir=GetPositionIK.Request()
        ir.ik_request.group_name='ur_arm'
        ir.ik_request.ik_link_name='tcp'
        ir.ik_request.robot_state=seed
        ir.ik_request.pose_stamped=fk.pose_stamped[0]
        ir.ik_request.avoid_collisions=avoid
        ir.ik_request.timeout.sec=1
        answer=call(GetPositionIK,'/compute_ik',ir)
        result['ik'].append({'seed':label,'avoid_collisions':avoid,'result':to_dict(answer)})
print(json.dumps(result,separators=(',',':')))
node.destroy_node()
rclpy.shutdown()
