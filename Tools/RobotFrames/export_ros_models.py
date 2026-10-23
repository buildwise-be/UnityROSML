#!/usr/bin/env python3
"""Expand the real ROS Xacro model and export calibrated, Unity-readable data.

Run with xacro, PyYAML and ament_index_python on PYTHONPATH. No ROS nodes are
started. Source packages stay read-only; generated control wrappers/package
indexes live in a TemporaryDirectory. All output poses are ROS FLU, metres,
quaternions XYZW, and joint positions/limits radians.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

from check_robot_frames import rotation_axis, identity, multiply, transform


TOOLS = ("none", "kenos", "kenos_sl")
BASE_LINK = "ur_base_link"


def numbers(text, count, default=None):
    values = [float(value) for value in text.split()] if text else list(default or [0.0] * count)
    if len(values) != count or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Expected {count} finite values: {text!r}")
    return values


def matrix_pose(matrix):
    """Stable rotation-matrix to unit XYZW quaternion, including half turns."""
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
             (m[1][0] - m[0][1]) / s, s / 4]
    else:
        i = max(range(3), key=lambda index: m[index][index])
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0.0, 1 + m[i][i] - m[j][j] - m[k][k])) * 2
        q = [0.0] * 4
        q[i] = s / 4
        q[j] = (m[j][i] + m[i][j]) / s
        q[k] = (m[k][i] + m[i][k]) / s
        q[3] = (m[k][j] - m[j][k]) / s
    length = math.sqrt(sum(value * value for value in q))
    q = [value / length for value in q]
    if q[3] < 0:
        q = [-value for value in q]
    return {"position": [m[i][3] for i in range(3)], "rotation": q}


def origin_matrix(element):
    origin = element.find("origin")
    if origin is None:
        return identity()
    return transform(numbers(origin.get("xyz"), 3), numbers(origin.get("rpy"), 3))


def geometry_record(element, material_colors):
    geometry = element.find("geometry")
    if geometry is None or len(geometry) != 1:
        raise ValueError("A visual/collision must have exactly one geometry")
    shape = geometry[0]
    kind = shape.tag
    if kind not in ("box", "cylinder", "sphere", "mesh"):
        raise ValueError(f"Unsupported URDF geometry {kind}")
    result = {"kind": kind, **matrix_pose(origin_matrix(element)),
              "size": [0.0] * 3, "radius": 0.0, "length": 0.0,
              "meshUri": "", "scale": [1.0] * 3, "color": [0.7, 0.7, 0.7, 1.0]}
    if kind == "box":
        result["size"] = numbers(shape.get("size"), 3)
    elif kind in ("cylinder", "sphere"):
        result["radius"] = float(shape.get("radius"))
        if kind == "cylinder":
            result["length"] = float(shape.get("length"))
    else:
        result["meshUri"] = shape.get("filename")
        result["scale"] = numbers(shape.get("scale"), 3, [1.0] * 3)
    material = element.find("material")
    if material is not None:
        color = material.find("color")
        if color is not None:
            result["color"] = numbers(color.get("rgba"), 4)
        elif material.get("name") in material_colors:
            result["color"] = material_colors[material.get("name")]
    return result


def parse_model(xml_text, tool, calibration_hash):
    root = ET.fromstring(xml_text)
    source_links = {link.get("name"): link for link in root.findall("link")}
    source_joints = root.findall("joint")
    if len(source_links) != len(root.findall("link")):
        raise ValueError("Duplicate URDF link name")
    colors = {m.get("name"): numbers(m.find("color").get("rgba"), 4)
              for m in root.findall("material") if m.find("color") is not None}
    children = {}
    for joint in source_joints:
        child = joint.find("child").get("link")
        if child in children:
            raise ValueError(f"Multiple parents for {child}")
        children[child] = joint

    ordered_names = [BASE_LINK]
    for name in ordered_names:
        ordered_names.extend(joint.find("child").get("link") for joint in source_joints
                             if joint.find("parent").get("link") == name)
        if len(ordered_names) != len(set(ordered_names)):
            raise ValueError("Cycle in URDF model")
    if BASE_LINK not in source_links or "tcp" not in ordered_names:
        raise ValueError("Missing robot base or TCP")

    links, origins = [], {}
    for name in ordered_names:
        joint = children.get(name) if name != BASE_LINK else None
        local = origin_matrix(joint) if joint is not None else identity()
        origins[name] = local
        axis = joint.find("axis") if joint is not None else None
        limit = joint.find("limit") if joint is not None else None
        source = source_links[name]
        links.append({
            "name": name, "parent": joint.find("parent").get("link") if joint is not None else "",
            "jointName": joint.get("name") if joint is not None else "",
            "jointType": joint.get("type") if joint is not None else "fixed",
            **matrix_pose(local), "axis": numbers(axis.get("xyz"), 3) if axis is not None else [1.0, 0.0, 0.0],
            "lower": float(limit.get("lower", "0")) if limit is not None else 0.0,
            "upper": float(limit.get("upper", "0")) if limit is not None else 0.0,
            "visuals": [geometry_record(visual, colors) for visual in source.findall("visual")],
            "collisions": [geometry_record(collision, colors) for collision in source.findall("collision")],
        })
    model = {"tool": tool, "calibrationHash": calibration_hash, "links": links}
    model["goldenCases"] = golden_cases(links, origins)
    return model


def golden_cases(links, origins):
    movable = [link for link in links if link["jointType"] in ("revolute", "continuous")]
    if len(movable) != 6:
        raise ValueError(f"Expected six arm joints, found {len(movable)}")
    names = [link["jointName"] for link in movable]
    configurations = [("zero", [0.0] * 6),
                      ("home", [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]),
                      ("bent", [0.6, -1.2, 1.5, -0.8, 1.1, -0.4])]
    for i in range(6):
        positions = [0.0] * 6
        positions[i] = math.pi / 2
        configurations.append((f"joint_{i + 1}_quarter_turn", positions))
    generator = random.Random(160316)
    for i in range(12):
        configurations.append((f"random_{i + 1:02}", [generator.uniform(-math.pi, math.pi) for _ in names]))
    result = []
    for name, positions in configurations:
        by_joint = dict(zip(names, positions))
        world = {}
        output = []
        for link in links:
            local = origins[link["name"]]
            if link["jointType"] in ("revolute", "continuous"):
                local = multiply(local, rotation_axis(link["axis"], by_joint[link["jointName"]]))
            elif link["jointType"] != "fixed":
                raise ValueError(f"Unsupported joint type {link['jointType']}")
            world[link["name"]] = multiply(world[link["parent"]], local) if link["parent"] else local
            output.append({"name": link["name"], **matrix_pose(world[link["name"]])})
        result.append({"name": name, "jointNames": names, "jointPositions": positions, "links": output})
    return result


def package_roots(ros_root, dependency_root):
    packages = {}
    for package_xml in sorted((ros_root / "ros_ws/src").glob("*/package.xml")):
        packages[ET.parse(package_xml).getroot().findtext("name")] = package_xml.parent
    packages["ur_description"] = dependency_root / "ur_description"
    packages["ur_robot_driver"] = dependency_root / "ur_driver/ur_robot_driver"
    for name, path in packages.items():
        if not path.is_dir():
            raise ValueError(f"Package {name} not found at {path}")
    return packages


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def revision(path):
    environment = {key: value for key, value in os.environ.items() if key not in ("GIT_DIR", "GIT_WORK_TREE")}
    process = subprocess.run(["git", "-C", str(path), "rev-parse", "HEAD"],
                             env=environment, text=True, capture_output=True)
    return process.stdout.strip() if process.returncode == 0 else "unavailable"


def export_models(ros_root, dependency_root, output_directory, expanded_directory):
    import xacro
    import yaml

    packages = package_roots(ros_root, dependency_root)
    source_packages = dict(packages)
    description = packages["ur16e_mir250_description"] / "urdf/mir250_ur16e.urdf.xacro"
    calibration = packages["ur16e_mir250_control"] / "config/ur16e_calibration.yaml"
    calibration_hash = yaml.safe_load(calibration.read_text())["kinematics"]["hash"]
    source_files = {description, calibration, Path(__file__).resolve(),
                    Path(__file__).with_name("check_robot_frames.py").resolve()}
    # Launch definitions establish which stored calibration/defaults the ROS consumers use.
    source_files.update((ros_root / "ros_ws/src").glob("*/launch/*launch.py"))
    models = []
    with tempfile.TemporaryDirectory(prefix="unity-robot-model-export-") as temporary:
        temp = Path(temporary)
        control_source = packages["ur16e_mir250_control"]
        control_share = temp / "control"
        control_share.mkdir()
        for child in control_source.iterdir():
            if child.name != "urdf":
                (control_share / child.name).symlink_to(child, target_is_directory=child.is_dir())
        (control_share / "urdf").mkdir()
        generator = control_source / "scripts/generate_control_xacro.py"
        template = control_source / "urdf/ur16e_mir250.urdf.xacro.in"
        driver_xacro = packages["ur_robot_driver"] / "urdf/ur.ros2_control.xacro"
        generated = control_share / "urdf/ur16e_mir250.urdf.xacro"
        subprocess.run([sys.executable, "-B", str(generator), "--driver-xacro", str(driver_xacro),
                        "--template", str(template), "--output", str(generated)], check=True)
        source_files.update((generator, template, driver_xacro))
        packages["ur16e_mir250_control"] = control_share
        prefix = temp / "prefix"
        for name, package in packages.items():
            resource = prefix / "share/ament_index/resource_index/packages" / name
            resource.parent.mkdir(parents=True, exist_ok=True)
            resource.touch()
            share = prefix / "share" / name
            share.symlink_to(package, target_is_directory=True)
        old_prefix = os.environ.get("AMENT_PREFIX_PATH")
        os.environ["AMENT_PREFIX_PATH"] = str(prefix)
        try:
            for tool in TOOLS:
                xacro.all_includes.clear()
                document = xacro.process_file(str(description), mappings={
                    "tool": tool, "ur_type": "ur16e", "use_control": "false",
                    "kinematics_params": str(calibration),
                })
                xml_text = document.toxml()
                model = parse_model(xml_text, tool, calibration_hash)
                model["expandedUrdfSha256"] = hashlib.sha256(xml_text.encode()).hexdigest()
                models.append(model)
                expanded_directory.mkdir(parents=True, exist_ok=True)
                (expanded_directory / f"{tool}.urdf.txt").write_text(xml_text + "\n", encoding="utf-8")
                for included in xacro.all_includes:
                    included_path = Path(included).resolve()
                    if included_path == generated.resolve():
                        continue
                    if included_path.is_file():
                        source_files.add(included_path)
        finally:
            if old_prefix is None:
                os.environ.pop("AMENT_PREFIX_PATH", None)
            else:
                os.environ["AMENT_PREFIX_PATH"] = old_prefix

    # Add any locally available mesh bytes. Sparse upstream checkouts may omit
    # meshes: their authoritative URI remains in the model without a false hash.
    missing_meshes = set()
    for model in models:
        for link in model["links"]:
            for geometry in link["visuals"] + link["collisions"]:
                uri = geometry["meshUri"]
                if not uri:
                    continue
                if not uri.startswith("package://"):
                    raise ValueError(f"Expected package mesh URI, got {uri}")
                package, relative = uri[len("package://"):].split("/", 1)
                mesh_path = source_packages[package] / relative
                if mesh_path.is_file():
                    source_files.add(mesh_path.resolve())
                else:
                    missing_meshes.add(uri)

    def portable_path(path):
        path = path.resolve()
        project = Path(__file__).resolve().parents[2]
        if path.is_relative_to(project):
            return "unity/" + path.relative_to(project).as_posix()
        if path.is_relative_to(ros_root.resolve()):
            return "ros/" + path.relative_to(ros_root.resolve()).as_posix()
        for package, directory in sorted(source_packages.items(), key=lambda pair: len(str(pair[1])), reverse=True):
            if path.is_relative_to(directory.resolve()):
                return package + "/" + path.relative_to(directory.resolve()).as_posix()
        if path.is_relative_to((dependency_root / "xacro").resolve()):
            return "xacro/" + path.relative_to((dependency_root / "xacro").resolve()).as_posix()
        raise ValueError(f"Source provenance not mapped: {path}")

    source_files.update((dependency_root / "xacro/xacro").glob("*.py"))
    result = {
        "schemaVersion": 1, "baseLink": BASE_LINK,
        "convention": "ROS FLU; metres; XYZW quaternion; radians; origin * axis rotation; base-relative golden poses",
        "generation": {"pythonVersion": sys.version.split()[0], "yamlVersion": yaml.__version__,
                       "armCalibrationFile": "ros/ros_ws/src/ur16e_mir250_control/config/ur16e_calibration.yaml",
                       "urType": "ur16e", "useControl": False},
        "sourceFiles": sorted([{"path": portable_path(path), "sha256": sha256(path)} for path in source_files],
                              key=lambda item: item["path"]),
        "dependencies": [{"name": name, "revision": revision(dependency_root / directory)}
                         for name, directory in (("ur_description", "ur_description"), ("xacro", "xacro"),
                                                 ("ament_index", "ament_index"), ("ur_driver", "ur_driver"))],
        "unavailableMeshUris": sorted(missing_meshes),
        "models": models,
    }
    output_directory.mkdir(parents=True, exist_ok=True)
    output = output_directory / "Models.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return result, output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-root", type=Path, required=True)
    parser.add_argument("--dependency-root", type=Path, required=True,
                        help="Existing offline source directory containing xacro, ament_index, ur_description and ur_driver")
    project = Path(__file__).resolve().parents[2]
    parser.add_argument("--output-directory", type=Path, default=project / "Assets/Resources/RobotModels")
    parser.add_argument("--expanded-directory", type=Path,
                        help="Defaults to output-directory; canonical XML is retained as <tool>.urdf.txt")
    args = parser.parse_args()
    result, path = export_models(args.ros_root, args.dependency_root, args.output_directory,
                                args.expanded_directory or args.output_directory)
    print(f"Exported {path}")
    for model in result["models"]:
        print(f"{model['tool']}: {len(model['links'])} links, {len(model['goldenCases'])} complete FK cases")


if __name__ == "__main__":
    main()
