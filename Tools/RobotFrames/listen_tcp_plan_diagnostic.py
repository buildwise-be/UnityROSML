"""Passively capture a user-requested TCP plan and query FK at reception."""
import copy
import json
import time
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionFK
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict as to_dict

rclpy.init()
node = rclpy.create_node('codex_passive_tcp_plan_recorder')
fk_client = node.create_client(GetPositionFK,'/compute_fk')
fk_client.wait_for_service(timeout_sec=5)
latest = {}
pending = []
seen_request = None
end_after = None
def emit(kind, data):
    print(json.dumps({'kind':kind,'received_unix':time.time(),'data':data},
                     separators=(',',':')),flush=True)
def joint_state(msg):
    latest['joints'] = copy.deepcopy(msg)
def controller(msg):
    latest['controller_tcp'] = copy.deepcopy(msg)
def target(msg):
    global seen_request, end_after
    seen_request = str(msg.header.stamp.sec)+':'+str(msg.header.stamp.nanosec)
    end_after = time.monotonic()+30
    emit('tcp_plan_request',{'request':to_dict(msg),
         'controller_tcp':to_dict(latest['controller_tcp']) if 'controller_tcp' in latest else None})
    if 'joints' in latest:
        request = GetPositionFK.Request()
        request.header.frame_id = msg.header.frame_id
        request.fk_link_names = ['ur_base_link','ur_flange','ur_tool0','tcp']
        request.robot_state.joint_state = latest['joints']
        emit('joint_snapshot',to_dict(latest['joints']))
        pending.append((seen_request,fk_client.call_async(request)))
def status(msg):
    global end_after
    if seen_request is None:
        return
    try:
        value = json.loads(msg.data)
    except Exception:
        value = msg.data
    emit('tcp_planning_status',value)
    if isinstance(value,dict) and value.get('phase') in ['failed','ready','invalidated']:
        if value.get('request_id') == seen_request:
            end_after = time.monotonic()+3
qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
subscriptions = [
    node.create_subscription(JointState,'/joint_states',joint_state,qos),
    node.create_subscription(PoseStamped,'/tcp_pose_broadcaster/pose',controller,qos),
    node.create_subscription(PoseArray,'/tcp_poses_plan_target',target,10),
    node.create_subscription(String,'/mtc/tcp_planning_status',status,10)]
emit('listener_ready',{'duration_seconds':180,'publishes':False})
deadline = time.monotonic()+180
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=0.1)
    for token,future in list(pending):
        if future.done():
            emit('current_fk_at_request',{'request_id':token,'response':to_dict(future.result())})
            pending.remove((token,future))
    if end_after is not None and time.monotonic()>end_after:
        break
emit('listener_finished',{'captured_request_id':seen_request})
node.destroy_node()
rclpy.shutdown()
