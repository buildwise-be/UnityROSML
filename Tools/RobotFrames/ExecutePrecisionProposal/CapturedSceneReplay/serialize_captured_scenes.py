"""Serialize saved ROS scene dictionaries; no rclpy.init, nodes or graph calls."""
import json
import sys
from pathlib import Path

from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.serialization import serialize_message
from rosidl_runtime_py.set_message import set_message_fields


def restore_operation_bytes(value):
    # message_to_ordereddict represents octet CollisionObject.operation as text.
    if isinstance(value, dict):
        for key, item in value.items():
            if key == "operation" and isinstance(item, str):
                value[key] = (item.encode("latin-1") if isinstance(CollisionObject().operation, bytes)
                              else ord(item))
            else:
                restore_operation_bytes(item)
    elif isinstance(value, list):
        for item in value:
            restore_operation_bytes(item)


snapshot = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8-sig"))
output = Path(sys.argv[2])
for name, saved in (("planned", snapshot["solutions"]["/solution"]["start_scene"]),
                    ("current", snapshot["current_scene"])):
    restore_operation_bytes(saved)
    scene = PlanningScene()
    set_message_fields(scene, saved)
    data = serialize_message(scene)
    (output / (name + ".cdr")).write_bytes(data)
    print(f"Serialized {name}: {len(data)} bytes; {len(scene.world.collision_objects)} world objects")
