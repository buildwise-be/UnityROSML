#!/usr/bin/env python3
"""Print offline ROS2 CDR fixtures; run in an existing sourced ROS installation.

Uses installed rcl_interfaces and rclpy.serialization only. Does not initialize
rclpy, create a node, send a request, or contact the ROS graph.
"""

import base64
import hashlib
import json
import platform
import struct

from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.serialization import serialize_message


DESCRIPTION = '<robot name="ur16e_μ"><link name="ur_base_link"/>\n<link name="tcp"/></robot>'
request = GetParameters.Request(names=["robot_description"])
unicode_request = GetParameters.Request(names=["robot_description", "odd", "π"])
response = GetParameters.Response(values=[
    ParameterValue(type=4, string_value=DESCRIPTION),
    ParameterValue(type=9, bool_value=True, integer_value=-123456789012345,
                   double_value=-1234.125, string_value="alignment_π",
                   byte_array_value=[bytes([value]) for value in (0, 1, 127, 128, 255)],
                   bool_array_value=[True, False, True],
                   integer_array_value=[-9223372036854775808, 0, 9223372036854775807],
                   double_array_value=[0.0, -0.125, 1e100],
                   string_array_value=["", "A", "calibrated_μ", "<robot/>\n"]),
])


def record(name, message):
    payload = bytes(serialize_message(message))
    # Fast-CDR padding is unspecified and can contain allocation bytes. Compare
    # actual field bytes while canonicalizing only schema-proven alignment gaps.
    offset, padding = 4, []

    def align(size):
        nonlocal offset
        count = (-offset + 4) % size
        padding.extend(range(offset, offset + count))
        offset += count

    def scalar(size):
        nonlocal offset
        align(size)
        offset += size

    def length():
        nonlocal offset
        align(4)
        count = struct.unpack_from("<I", payload, offset)[0]
        offset += 4
        return count

    def string():
        nonlocal offset
        count = length()
        offset += count

    def sequence(size):
        nonlocal offset
        count = length()
        if count:
            align(size)
            offset += count * size

    if name.endswith("request"):
        for _ in range(length()):
            string()
    else:
        for _ in range(length()):
            scalar(1)  # type
            scalar(1)  # bool
            scalar(8)  # integer
            scalar(8)  # double
            string()
            for size in (1, 1, 8, 8):
                sequence(size)
            for _ in range(length()):
                string()
    if offset != len(payload):
        raise ValueError(f"Fixture schema consumed {offset} of {len(payload)} bytes")
    canonical = bytearray(payload)
    for index in padding:
        canonical[index] = 0
    return {"name": name, "base64": base64.b64encode(payload).decode(),
            "canonicalBase64": base64.b64encode(canonical).decode(), "paddingOffsets": padding,
            "sha256": hashlib.sha256(payload).hexdigest(), "length": len(payload)}


print(json.dumps({
    "schemaVersion": 1,
    "origin": "Installed ROS2 rcl_interfaces + rclpy.serialization.serialize_message; no rclpy.init or ROS graph calls",
    "pythonVersion": platform.python_version(),
    "description": DESCRIPTION,
    "fixtures": [record("request", request), record("unicode_request", unicode_request), record("response", response)],
}, ensure_ascii=True, indent=2))
