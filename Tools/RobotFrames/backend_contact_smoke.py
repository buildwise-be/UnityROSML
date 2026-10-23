"""Offline-only MTC contact API smoke, run in Docker --network none.

Publishes a planning request to a local child process, never an Execute request.
The planning scene services and joint feedback are in-memory fixtures.
"""
import copy
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import PlanningScene, RobotTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ur16e_interfaces.msg import PickPlacePoses
import yaml

ROOT = Path('/tmp/tools')
OUT = Path('/tmp/result')
MODEL = json.loads((ROOT / 'planning_models_diagnostic_20261022.json').read_text())
EVENTS = [json.loads(line) for line in (ROOT / 'fresh_tcp_plan_capture_20261022.jsonl').read_text().splitlines()]
CONFIG = Path('/home/ros_ws/install/ur16e_mir250_moveit_config/share/ur16e_mir250_moveit_config/config')


def fixture(kind):
    return next(event['data'] for event in EVENTS if event['kind'] == kind)


def decode_fixture(value):
    # The read-only diagnostic serializer recorded ROS uint8 as one-byte text.
    if isinstance(value, dict):
        return {key: decode_fixture(item) for key, item in value.items()}
    if isinstance(value, list):
        return [decode_fixture(item) for item in value]
    if isinstance(value, str) and len(value) == 1 and ord(value) < 32:
        return ord(value)
    return value


class MockScene(Node):
    def __init__(self):
        super().__init__('offline_contact_smoke')
        self.scene = PlanningScene()
        scene = json.loads((ROOT / 'planning_scene_diagnostic_20261022.json').read_text())['scene']
        scene['world']['collision_objects'] = [obj for obj in scene['world']['collision_objects'] if obj['id'] != 'object']
        scene['robot_state']['attached_collision_objects'] = []
        scene['robot_state']['joint_state'] = fixture('joint_snapshot')
        set_message_fields(self.scene, decode_fixture(scene))
        self.scene.is_diff = False
        self.applied = []
        self.statuses = []
        self.execution_statuses = []
        self.trajectories = []
        self.tcp_statuses = []
        self.tcp_trajectories = []
        self.create_service(GetPlanningScene, '/get_planning_scene', self.get_scene)
        self.create_service(ApplyPlanningScene, '/apply_planning_scene', self.apply_scene)
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mtc/planning_status', lambda message: self.statuses.append(json.loads(message.data)), qos)
        self.create_subscription(String, '/mtc/execution_status', lambda message: self.execution_statuses.append(message.data), qos)
        self.create_subscription(RobotTrajectory, '/mtc/planned_trajectory', lambda message: self.trajectories.append(len(message.joint_trajectory.points)), qos)
        self.create_subscription(String, '/mtc/tcp_planning_status', lambda message: self.tcp_statuses.append(json.loads(message.data)), qos)
        self.create_subscription(RobotTrajectory, '/mtc/tcp_planned_trajectory', lambda message: self.tcp_trajectories.append(len(message.joint_trajectory.points)), qos)
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.02, self.publish_joints)
        self.targets = self.create_publisher(PickPlacePoses, '/pick_place_tcp_target', 10)
        self.tcp_targets = self.create_publisher(PoseArray, '/tcp_poses_plan_target', 10)
        self.capability = self.create_client(GetParameters, '/mtc_test_task_node/get_parameters')

    def publish_joints(self):
        message = copy.deepcopy(self.scene.robot_state.joint_state)
        message.header.stamp = self.get_clock().now().to_msg()
        self.joints.publish(message)

    def get_scene(self, request, response):
        response.scene = copy.deepcopy(self.scene)
        return response

    def apply_scene(self, request, response):
        for obj in request.scene.world.collision_objects:
            operation = obj.operation[0] if isinstance(obj.operation, bytes) else obj.operation
            self.applied.append({'id': obj.id, 'operation': operation})
            self.scene.world.collision_objects = [current for current in self.scene.world.collision_objects if current.id != obj.id]
            if obj.operation != obj.REMOVE:
                self.scene.world.collision_objects.append(copy.deepcopy(obj))
        response.success = True
        return response


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'This test must stay isolated.'
    description = MODEL['/robot_state_publisher/get_parameters']['response']['values'][0]['string_value']
    semantic = MODEL['/move_group/get_parameters']['response']['values'][1]['string_value']
    ompl = yaml.safe_load(Path('/opt/ros/jazzy/share/moveit_configs_utils/default_configs/ompl_planning.yaml').read_text())
    ompl.update(yaml.safe_load((CONFIG / 'ompl_planning.yaml').read_text()))
    params = {'robot_description': description, 'robot_description_semantic': semantic,
              'tool': 'kenos_sl', 'ik_frame': 'tcp', 'pick_place_tcp_planning_enabled': True,
              'robot_description_kinematics': yaml.safe_load((CONFIG / 'kinematics.yaml').read_text()),
              'robot_description_planning': yaml.safe_load((CONFIG / 'joint_limits.yaml').read_text()),
              'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl}
    path = Path('/tmp/contact-smoke.yaml')
    path.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    rclpy.init()
    node = MockScene()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    log_path = OUT / 'contact_smoke.log'
    result = {'hardware': False, 'container_network': 'none', 'execute_published': False}
    with log_path.open('w') as log:
        process = subprocess.Popen(['/tmp/result/test_task', '--ros-args', '--params-file', str(path)], stdout=log, stderr=subprocess.STDOUT)
        try:
            if not node.capability.wait_for_service(timeout_sec=20):
                raise RuntimeError('New task node did not expose its parameter service')
            future = node.capability.call_async(GetParameters.Request(names=['pick_place_tcp_planning_enabled']))
            deadline = time.monotonic() + 10
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            result['capability'] = bool(future.result().values[0].bool_value)
            assert result['capability']
            deadline = time.monotonic() + 10
            while node.targets.get_subscription_count() == 0 and time.monotonic() < deadline:
                time.sleep(0.1)
            assert node.targets.get_subscription_count() > 0
            time.sleep(0.3)
            fk = fixture('current_fk_at_request')['response']['pose_stamped'][3]
            request = PickPlacePoses()
            set_message_fields(request.pick_pose, fk)
            q = request.pick_pose.pose.orientation
            normal = (2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y))
            for axis, delta in zip(('x', 'y', 'z'), normal):
                setattr(request.pick_pose.pose.position, axis, getattr(request.pick_pose.pose.position, axis) + 0.02 * delta)
            request.place_pose = copy.deepcopy(request.pick_pose)
            request.place_pose.pose.position.y += 0.02
            result['target_delta_metres'] = {'pick_along_tool_z': 0.02, 'place_base_y': 0.02}
            result['request_topic'] = '/pick_place_tcp_target'
            node.targets.publish(request)
            deadline = time.monotonic() + 100
            while time.monotonic() < deadline and process.poll() is None:
                if any(count > 0 for count in node.trajectories) or any(json.loads(status).get('status') == 'failed' for status in node.execution_statuses):
                    break
                time.sleep(0.1)
            time.sleep(0.5)
            result.update({'statuses': node.statuses, 'execution_statuses': node.execution_statuses,
                           'preview_trajectory_points': node.trajectories, 'mock_scene_updates': node.applied,
                           'process_exit_before_termination': process.poll()})
            if any(status.get('overall_success') for status in node.statuses):
                tcp_request = PoseArray()
                tcp_request.header.frame_id = 'ur_base_link'
                tcp_request.header.stamp = node.get_clock().now().to_msg()
                tcp_request.poses = [copy.deepcopy(request.pick_pose.pose), copy.deepcopy(request.place_pose.pose)]
                node.tcp_targets.publish(tcp_request)
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline and not any(status.get('phase') in ('ready', 'failed') for status in node.tcp_statuses):
                    time.sleep(0.1)
                time.sleep(0.3)
                result['tcp_after_contact'] = {'statuses': node.tcp_statuses, 'preview_points': node.tcp_trajectories,
                                                'remaining_world_objects': [obj.id for obj in node.scene.world.collision_objects]}
        except Exception as error:
            result['error'] = str(error)
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
    text = log_path.read_text()
    result['contact_place_stage_present'] = any(stage['name'] == 'place tool twist alternatives' for status in node.statuses for stage in status.get('stages', []))
    result['initialization_exception'] = 'Error initializing stages' in text or 'Error during init' in text
    result['full_solution'] = any(status.get('overall_success') for status in node.statuses)
    (OUT / 'contact_smoke.json').write_text(json.dumps(result, indent=2))
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({key: value for key, value in result.items() if key not in ('statuses', 'execution_statuses')}, indent=2))


if __name__ == '__main__':
    main()
