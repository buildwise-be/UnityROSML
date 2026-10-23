#!/usr/bin/env python3
"""Compare checked-in Unity geometry with ROS Kenos defaults, without ROS or Unity.

Only Python's standard library is used. This reads a restricted, numeric subset
of the ROS calibration YAML and the explicit Kenos Xacro macro; it does not run
Xacro, contact a robot, or claim to identify the physically mounted tool.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


XACRO = "{http://www.ros.org/wiki/xacro}"
ARM_SECTIONS = {
    "ur_shoulder_pan_joint": "shoulder",
    "ur_shoulder_lift_joint": "upper_arm",
    "ur_elbow_joint": "forearm",
    "ur_wrist_1_joint": "wrist_1",
    "ur_wrist_2_joint": "wrist_2",
    "ur_wrist_3_joint": "wrist_3",
}
SCALAR_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def identity():
    return [[float(row == column) for column in range(4)] for row in range(4)]


def multiply(a, b):
    return [[sum(a[row][k] * b[k][column] for k in range(4))
             for column in range(4)] for row in range(4)]


def rotation_axis(axis, angle):
    length = math.sqrt(sum(value * value for value in axis))
    if not math.isfinite(length) or length <= 0:
        raise ValueError("A movable joint must have a finite, nonzero axis")
    x, y, z = [value / length for value in axis]
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    result = identity()
    result[:3] = [
        [t*x*x+c, t*x*y-s*z, t*x*z+s*y, 0.0],
        [t*x*y+s*z, t*y*y+c, t*y*z-s*x, 0.0],
        [t*x*z-s*y, t*y*z+s*x, t*z*z+c, 0.0],
    ]
    return result


def transform(xyz, rpy):
    # URDF fixed-axis roll/pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll).
    roll, pitch, yaw = rpy
    result = multiply(multiply(rotation_axis((0, 0, 1), yaw),
                               rotation_axis((0, 1, 0), pitch)),
                      rotation_axis((1, 0, 0), roll))
    for index, value in enumerate(xyz):
        result[index][3] = value
    return result


def inverse(matrix):
    result = identity()
    for row in range(3):
        for column in range(3):
            result[row][column] = matrix[column][row]
        result[row][3] = -sum(matrix[k][row] * matrix[k][3] for k in range(3))
    return result


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def compare_transforms(unity, ros):
    delta = [ros[index][3] - unity[index][3] for index in range(3)]
    relative = multiply(inverse(unity), ros)
    # atan2 retains precision for very small rotation discrepancies.
    sine = 0.5 * norm([relative[2][1] - relative[1][2],
                       relative[0][2] - relative[2][0],
                       relative[1][0] - relative[0][1]])
    cosine = max(-1.0, min(1.0, (sum(relative[i][i] for i in range(3)) - 1) / 2))
    angle = math.atan2(sine, cosine)
    return {
        "ros_minus_unity_position_m": delta,
        "position_error_mm": norm(delta) * 1000,
        "orientation_error_degrees": math.degrees(angle),
    }


def transform_record(matrix):
    # Matrices avoid quaternion sign ambiguity when inspecting matching chains.
    return {"translation_m": [matrix[i][3] for i in range(3)],
            "rotation_matrix": [row[:3] for row in matrix[:3]]}


def numbers(text, count=3):
    values = [float(value) for value in text.split()]
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Expected {count} finite numbers, got {text!r}")
    return values


def parse_joint(element, substitute=lambda value: value):
    origin = element.find("origin")
    xyz = numbers(substitute(origin.get("xyz", "0 0 0"))) if origin is not None else [0.0] * 3
    rpy = numbers(substitute(origin.get("rpy", "0 0 0"))) if origin is not None else [0.0] * 3
    axis = element.find("axis")
    return {
        "name": substitute(element.attrib["name"]),
        "type": element.attrib["type"],
        "parent": substitute(element.find("parent").attrib["link"]),
        "child": substitute(element.find("child").attrib["link"]),
        "xyz_m": xyz,
        "rpy_rad": rpy,
        "axis": numbers(axis.get("xyz", "1 0 0")) if axis is not None else [1.0, 0.0, 0.0],
    }


def chain(joints, parent, child, require_fixed=False):
    by_child = {}
    for joint in joints:
        if joint["child"] in by_child:
            raise ValueError(f"Multiple parent joints for {joint['child']}")
        by_child[joint["child"]] = joint
    result, seen = [], set()
    while child != parent:
        if child in seen or child not in by_child:
            raise ValueError(f"No acyclic chain from {parent} to {child}")
        seen.add(child)
        joint = by_child[child]
        if require_fixed and joint["type"] != "fixed":
            raise ValueError(f"Tool chain contains movable joint {joint['name']}")
        result.append(joint)
        child = joint["parent"]
    return list(reversed(result))


def compose(joints, positions=None):
    positions = positions or {}
    result = identity()
    for joint in joints:
        result = multiply(result, transform(joint["xyz_m"], joint["rpy_rad"]))
        if joint["type"] in ("revolute", "continuous"):
            result = multiply(result, rotation_axis(joint["axis"], positions[joint["name"]]))
        elif joint["type"] != "fixed":
            raise ValueError(f"Unsupported joint type: {joint['type']}")
    return result


def compare_fixed_chains(unity, ros):
    same_topology = (len(unity) == len(ros) and
                     all((a["parent"], a["child"], a["type"]) ==
                         (b["parent"], b["child"], b["type"])
                         for a, b in zip(unity, ros)))
    comparisons = []
    for a, b in zip(unity, ros):
        comparisons.append({
            "unity_joint": a["name"], "ros_joint": b["name"],
            **compare_transforms(transform(a["xyz_m"], a["rpy_rad"]),
                                 transform(b["xyz_m"], b["rpy_rad"])),
        })
    matches = same_topology and all(
        item["position_error_mm"] <= 1e-6 and item["orientation_error_degrees"] <= 1e-6
        for item in comparisons)
    return {"same_topology": same_topology, "joint_comparisons": comparisons,
            "matches_old_ros_default": matches}


def estimated_tool0_tcp(unity_joints, depth_mm):
    """Compose a clearly provisional tool0 +Z depth through the real URDF frames."""
    if not math.isfinite(depth_mm) or depth_mm <= 0:
        raise ValueError("Estimated tool0 depth must be finite and greater than zero")
    flange_to_tool0 = compose(chain(unity_joints, "ur_flange", "ur_tool0", require_fixed=True))
    tool0_to_tcp = transform([0, 0, depth_mm / 1000], [0, 0, 0])
    candidate = multiply(flange_to_tool0, tool0_to_tcp)
    old_tcp = compose(chain(unity_joints, "ur_flange", "tcp", require_fixed=True))
    difference = compare_transforms(old_tcp, candidate)
    difference["candidate_minus_old_position_in_flange_m"] = difference.pop("ros_minus_unity_position_m")
    return {
        "status": "UNMEASURED_ESTIMATE_NOT_APPLIED",
        "estimated_tool0_depth_mm": depth_mm,
        "assumptions": [
            "The estimated thickness is provisionally treated as TCP distance along ur_tool0 +Z from the flange/tool0 origin.",
            "TCP orientation is provisionally identical to ur_tool0 (zero teach-pendant rotation).",
            "Body thickness may exclude adapter/mount depth and suction-pad compression; neither is measured here.",
            "Teach-pendant RX/RY/RZ use an axis-angle rotation vector, whereas URDF rpy uses fixed-axis roll/pitch/yaw; only the all-zero case is equated here.",
            "This candidate does not establish the physical TCP and is not applied to a URDF, prefab, runtime, or robot.",
        ],
        "flange_to_tool0_from_unity_urdf": transform_record(flange_to_tool0),
        "tool0_to_candidate_tcp": transform_record(tool0_to_tcp),
        "flange_to_candidate_tcp": transform_record(candidate),
        "candidate_minus_old_kenos": difference,
    }


def calibration_yaml(path):
    """Read the six known numeric blocks, failing on unsupported block content."""
    blocks, current, calibration_hash = {}, None, None
    for line_number, raw in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip() or line == "kinematics:":
            continue
        block = re.fullmatch(r"  ([A-Za-z_0-9]+):", line)
        scalar = re.fullmatch(r"    ([A-Za-z_0-9]+):\s*(\S+)", line)
        hash_match = re.fullmatch(r"  hash:\s*(\S+)", line)
        if block and block[1] in ARM_SECTIONS.values():
            current = block[1]
            if current in blocks:
                raise ValueError(f"Duplicate calibration block {current}")
            blocks[current] = {}
        elif scalar and current and scalar[1] in SCALAR_KEYS:
            if scalar[1] in blocks[current]:
                raise ValueError(f"Duplicate calibration scalar {current}.{scalar[1]}")
            value = float(scalar[2])
            if not math.isfinite(value):
                raise ValueError("Non-finite calibration scalar")
            blocks[current][scalar[1]] = value
        elif hash_match:
            calibration_hash = hash_match[1].strip("\"'")
            current = None
        else:
            raise ValueError(f"Unsupported calibration YAML at {path}:{line_number}: {raw}")
    for section in ARM_SECTIONS.values():
        if set(blocks.get(section, {})) != set(SCALAR_KEYS):
            raise ValueError(f"Incomplete calibration block: {section}")
    return blocks, calibration_hash


def ros_tool_chain(description_path, tool_path):
    description = ET.parse(description_path).getroot()
    defaults = {element.attrib["name"]: element.attrib["default"]
                for element in description.findall(XACRO + "arg")}
    tool_xyz = defaults["tool_tcp_xyz"]
    tool_rpy = defaults["tool_tcp_rpy"]
    substitutions = {"${parent}": "ur_flange", "${prefix}": "",
                     "${tcp_xyz}": tool_xyz, "${tcp_rpy}": tool_rpy}

    def substitute(value):
        for source, replacement in substitutions.items():
            value = value.replace(source, replacement)
        if "$" in value:
            raise ValueError(f"Unsupported Xacro expression: {value}")
        return value

    macros = ET.parse(tool_path).getroot().findall(XACRO + "macro")
    macro = next((entry for entry in macros if entry.get("name") == "kenos"), None)
    if macro is None:
        raise ValueError("Kenos macro not found")
    joints = [parse_joint(entry, substitute) for entry in macro.findall("joint")]
    return chain(joints, "ur_flange", "tcp", require_fixed=True), {
        "tool_tcp_xyz_m": numbers(tool_xyz), "tool_tcp_rpy_rad": numbers(tool_rpy),
        "description_default_tool": defaults["tool"],
    }


def fingerprint(path):
    return {"path": path.resolve().as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def build_report(unity_path, ros_root, estimated_tool0_depth_mm=None):
    source = ros_root / "ros_ws/src"
    description_path = source / "ur16e_mir250_description/urdf/mir250_ur16e.urdf.xacro"
    tool_path = source / "kenos-flexigrip/urdf/kenos.urdf.xacro"
    calibration_path = source / "ur16e_mir250_control/config/ur16e_calibration.yaml"
    unity_joints = [parse_joint(entry) for entry in ET.parse(unity_path).getroot().findall("joint")]
    unity_tool = chain(unity_joints, "ur_flange", "tcp", require_fixed=True)
    ros_tool, defaults = ros_tool_chain(description_path, tool_path)
    calibrated, calibration_hash = calibration_yaml(calibration_path)

    if set(ARM_SECTIONS) - {joint["name"] for joint in unity_joints}:
        raise ValueError("Unity URDF is missing expected UR arm joints")
    arm_differences, calibrated_joints = [], []
    for joint in unity_joints:
        replacement = dict(joint)
        if joint["name"] in ARM_SECTIONS:
            values = calibrated[ARM_SECTIONS[joint["name"]]]
            replacement["xyz_m"] = [values[key] for key in ("x", "y", "z")]
            replacement["rpy_rad"] = [values[key] for key in ("roll", "pitch", "yaw")]
            arm_differences.append({
                "joint": joint["name"], "unity_xyz_m": joint["xyz_m"],
                "ros_xyz_m": replacement["xyz_m"], "unity_rpy_rad": joint["rpy_rad"],
                "ros_rpy_rad": replacement["rpy_rad"],
                **compare_transforms(transform(joint["xyz_m"], joint["rpy_rad"]),
                                     transform(replacement["xyz_m"], replacement["rpy_rad"])),
            })
        calibrated_joints.append(replacement)

    unity_chain = chain(unity_joints, "ur_base_link", "tcp")
    # Isolate calibration effects. Fixed arm transforms/axes are inherited from
    # the Unity export because upstream ur_description is not vendored here.
    calibrated_chain = chain(calibrated_joints, "ur_base_link", "ur_flange") + ros_tool
    cases_degrees = {
        "zero": [0, 0, 0, 0, 0, 0],
        "home_like": [0, -90, 0, -90, 0, 0],
        "bent_forward": [0, -60, 90, -120, -90, 0],
        "side_bent": [90, -90, 90, -90, -90, 45],
        "folded": [-90, -120, 120, -90, 90, -45],
    }
    cases = []
    for name, degrees in cases_degrees.items():
        positions = {joint: math.radians(angle) for joint, angle in zip(ARM_SECTIONS, degrees)}
        unity_fk, ros_fk = compose(unity_chain, positions), compose(calibrated_chain, positions)
        cases.append({"name": name, "joint_positions_degrees": degrees,
                      "unity_base_to_tcp": transform_record(unity_fk),
                      "calibrated_proxy_base_to_tcp": transform_record(ros_fk),
                      **compare_transforms(unity_fk, ros_fk)})

    tool_delta = compare_transforms(compose(unity_tool), compose(ros_tool))
    report = {
        "schema_version": 1,
        "scope": "Offline source comparison of the old/default Kenos model; physical replacement tool is unmeasured.",
        "physical_tool_status": "MEASUREMENT_REQUIRED: no dimensions or TCP for the replacement Kenos variant were inferred.",
        "inputs": {"unity_combined_urdf": fingerprint(unity_path),
                   "ros_description_xacro": fingerprint(description_path),
                   "ros_kenos_xacro": fingerprint(tool_path),
                   "ros_arm_calibration": fingerprint(calibration_path)},
        "ros_defaults": defaults,
        "ros_calibration_hash": calibration_hash,
        "fixed_tool_comparison": {
            "parent_frame": "ur_flange", "child_frame": "tcp",
            **compare_fixed_chains(unity_tool, ros_tool),
            "unity_joints": unity_tool, "ros_default_joints": ros_tool,
            "unity_flange_to_tcp": transform_record(compose(unity_tool)),
            "ros_default_flange_to_tcp": transform_record(compose(ros_tool)),
            **tool_delta,
        },
        "arm_origin_differences": arm_differences,
        "forward_kinematics_comparison": {
            "joint_order": list(ARM_SECTIONS),
            "method": "URDF origin * joint-axis rotation; calibrated proxy replaces six arm origins and uses ROS default tool chain.",
            "samples": cases,
            "maximum_sampled_position_error_mm": max(case["position_error_mm"] for case in cases),
            "maximum_sampled_orientation_error_degrees": max(case["orientation_error_degrees"] for case in cases),
        },
        "limitations": [
            "Reads checked-in combined URDF, not Unity serialized prefabs or live ArticulationBody transforms.",
            "Evaluates only the known explicit numeric Kenos macro/defaults; launch-time TCP overrides are not detected.",
            "Calibration proxy inherits fixed arm transforms and joint axes from the Unity URDF; it is not a separately expanded deployed ROS model.",
            "Sampled FK errors are illustrative, not a global bound; joint samples are not validated for collisions, limits, reachability, or execution.",
            "Meshes, payload, physical mounting, replacement tool dimensions and real TCP calibration are not verified.",
            "Changing a tool model requires coordinated ROS model generation and Unity reimport; this script changes neither.",
        ],
        "ros_configuration_locations": {
            "select_tool": "ros_ws/src/ur16e_bringup/launch/ur16e_start.launch.py (tool argument; none or kenos)",
            "tool_tcp_defaults": "ros_ws/src/ur16e_mir250_description/urdf/mir250_ur16e.urdf.xacro (tool_tcp_xyz, tool_tcp_rpy)",
            "tool_mount_and_geometry": "ros_ws/src/kenos-flexigrip/urdf/kenos.urdf.xacro (mount/body transforms and meshes)",
            "arm_calibration": "ros_ws/src/ur16e_mir250_control/config/ur16e_calibration.yaml",
            "unity_export": "ros_ws/src/ur16e_mir250_description/scripts/generate_urdf.sh (pass matching kinematics_params and measured tool arguments)",
        },
    }
    if estimated_tool0_depth_mm is not None:
        report["provisional_tcp_estimate"] = estimated_tool0_tcp(unity_joints, estimated_tool0_depth_mm)
    return report


def main():
    project_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-root", required=True, type=Path)
    parser.add_argument("--unity-urdf", type=Path,
                        default=project_root / "Assets/Prefabs/ur16e_kenos/mir250_ur16e.urdf")
    parser.add_argument("--report", type=Path,
                        default=Path(__file__).with_name("reference_comparison.json"))
    parser.add_argument("--estimated-tool0-depth-mm", type=float,
                        help="Unmeasured candidate: tool0 +Z depth in mm with zero tool0-relative rotation; report only.")
    args = parser.parse_args()
    report = build_report(args.unity_urdf, args.ros_root, args.estimated_tool0_depth_mm)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Report: {args.report.resolve()}")
    print(f"Old/default Kenos tool chain matches: {report['fixed_tool_comparison']['matches_old_ros_default']}")
    fk = report["forward_kinematics_comparison"]
    print(f"Maximum position difference across five calibration-only FK samples: {fk['maximum_sampled_position_error_mm']:.3f} mm")
    print(report["physical_tool_status"])
    if "provisional_tcp_estimate" in report:
        provisional = report["provisional_tcp_estimate"]
        difference = provisional["candidate_minus_old_kenos"]
        print(f"PROVISIONAL ONLY: {provisional['estimated_tool0_depth_mm']:.1f} mm along tool0 +Z, zero relative rotation")
        print(f"Versus old Kenos TCP: {difference['position_error_mm']:.3f} mm position, "
              f"{difference['orientation_error_degrees']:.6f} degrees orientation difference; not applied")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, ET.ParseError) as error:
        print(f"Comparison failed: {error}", file=sys.stderr)
        sys.exit(1)
