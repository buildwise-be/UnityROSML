#!/usr/bin/env python3
"""Print a ROS 2 CDR interoperability fixture inside the pinned backend image.

Requires only installed ROS Python message packages; creates no node or ROS
transport. See RosMessageSchemas/manifest.json for the verified image digest.
"""

import base64
from rclpy.serialization import serialize_message
from geometry_msgs.msg import Point32, Pose, Quaternion
from moveit_msgs.msg import CollisionObject
from moveit_task_constructor_msgs.msg import Solution, SubTrajectory
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, UVCoordinate


solution = Solution()
solution.task_id = "fixture-cdr-plan-17"
solution.start_scene.name = solution.task_id
solution.start_scene.robot_model_name = "ur16e-fixture-cdr"
solution.start_scene.robot_state.joint_state.name = ["ur_shoulder_pan_joint"]
solution.start_scene.robot_state.joint_state.position = [0.125]
tile = CollisionObject()
tile.id = "tile-cdr-3"
tile.header.frame_id = "ur_base_link"
tile.pose.position.x = 0.321
tile.pose.orientation.w = 1.0
shape = SolidPrimitive()
shape.type = SolidPrimitive.BOX
shape.dimensions = [0.2, 0.2, 0.01]
# Polygon is a trailing Jazzy field even when the primitive is a box.
shape.polygon.points = [Point32(x=0.1, y=0.2, z=0.3), Point32(x=-0.4, y=0.5, z=0.6)]
tile.primitives = [shape]
tile.primitive_poses = [Pose(orientation=Quaternion(w=1.0))]
tile.subframe_names = ["contact"]
tile.subframe_poses = [Pose(orientation=Quaternion(w=1.0))]
solution.start_scene.world.collision_objects = [tile]

segment = SubTrajectory()
segment.info.id = 41
segment.info.stage_id = 73
segment.info.cost = 1.25
segment.info.comment = "vacuum off; settle 0.75 s"
segment.info.planner_id = "fixture-cdr-planner"
marker = Marker()
marker.header.frame_id = "ur_base_link"
marker.ns = "fixture-cdr-marker"
marker.id = 92
marker.pose.orientation.w = 1.0
marker.scale.x, marker.scale.y, marker.scale.z = 0.2, 0.3, 0.4
marker.color.r, marker.color.a = 0.75, 0.5
marker.texture_resource = "embedded://checker"
marker.texture.format = "png"
marker.texture.data = [137, 80, 78, 71]
marker.uv_coordinates = [UVCoordinate(u=0.25, v=0.75)]
marker.text = "fixture marker after UV fields"
marker.mesh_resource = "package://fixture/mesh.stl"
marker.mesh_file.filename = "triangle.stl"
marker.mesh_file.data = [1, 3, 5, 7, 9]
marker.mesh_use_embedded_materials = True
segment.info.markers = [marker]
segment.execution_info.controller_names = ["scaled_joint_trajectory_controller"]
segment.scene_diff.is_diff = True
segment.scene_diff.name = "after-cdr-settling"
solution.sub_trajectory = [segment]
print(base64.b64encode(serialize_message(solution)).decode())
