#!/usr/bin/env python3
"""Check Kenos SL mounting defaults across a ROS source tree and Unity catalog.

Uses the actual Python launch AST and Xacro XML without importing ROS launch or
starting nodes. This complements, rather than replaces, launch integration tests.
"""

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[2]
HEIGHT = 0.05325
LAUNCHES = (
    "ur16e_bringup/launch/ur16e_start.launch.py",
    "ur16e_mir250_control/launch/start_robot.launch.py",
    "ur16e_mir250_control/launch/urmir_rsp.launch.py",
    "ur16e_mir250_control/launch/urmir_rsp_kenos_sl.launch.py",
    "ur16e_mir250_moveit_config/launch/move_group.launch.py",
    "ur16e_mtc_gpio_stage/launch/mtc_commander.launch.py",
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_source(root):
    source = root / "ros_ws/src"
    inspected = []
    for relative in LAUNCHES:
        path = source / relative
        found = []
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            name = getattr(node.func, "id", getattr(node.func, "attr", ""))
            if name != "DeclareLaunchArgument" or not isinstance(node.args[0], ast.Constant):
                continue
            if node.args[0].value == "kenos_sl_mount_height":
                found.extend(ast.literal_eval(item.value) for item in node.keywords if item.arg == "default_value")
        if len(found) != 1 or float(found[0]) != HEIGHT:
            raise ValueError(f"Incorrect or missing mount launch default: {path}: {found}")
        inspected.append(path)

    macro = source / "kenos-flexigrip/urdf/kenos_sl.urdf.xacro"
    tree = ET.parse(macro)
    node = tree.getroot().find("{http://www.ros.org/wiki/xacro}macro[@name='kenos_sl']")
    if node is None or f"mount_height:={HEIGHT}" not in node.get("params", "").split():
        raise ValueError("Tool macro mount default differs: " + str(macro))
    inspected.append(macro)
    arm = source / "ur16e_mir250_description/include/bw_ur16e_macro.xacro"
    macros = [node for node in ET.parse(arm).getroot() if node.tag.rsplit("}", 1)[-1] == "macro"]
    if not any(f"kenos_sl_mount_height:={HEIGHT}" in node.get("params", "").split() for node in macros):
        raise ValueError("Arm macro mount default differs: " + str(arm))
    inspected.append(arm)
    description = source / "ur16e_mir250_description/urdf/mir250_ur16e.urdf.xacro"
    argument = ET.parse(description).getroot().find("{http://www.ros.org/wiki/xacro}arg[@name='kenos_sl_mount_height']")
    if argument is None or float(argument.get("default")) != HEIGHT:
        raise ValueError("Description mount default differs: " + str(description))
    inspected.append(description)
    return [{"path": path.relative_to(root).as_posix(), "sha256": digest(path)} for path in inspected]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-root", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    models = json.loads((PROJECT / "Assets/Resources/RobotModels/Models.json").read_text())
    model = next(item for item in models["models"] if item["tool"] == "kenos_sl")
    links = {link["name"]: link for link in model["links"]}
    if links["kenos_sl_base"]["position"] != [0, 0, HEIGHT]:
        raise ValueError("Unity tool attachment differs from ROS default")
    for shape in links["kenos_sl_mount"]["visuals"] + links["kenos_sl_mount"]["collisions"]:
        if shape["kind"] != "cylinder" or shape["length"] != HEIGHT or shape["position"] != [0, 0, HEIGHT / 2]:
            raise ValueError("Unity adapter geometry differs from ROS default")
    if links["tcp"]["position"] != [0.034, 0, 0.08975]:
        raise ValueError("Tool-local TCP changed unexpectedly")
    report = {
        "schemaVersion": 1, "completedUtc": datetime.now(timezone.utc).isoformat(),
        "mountHeightM": HEIGHT, "axialFlangeToTcpM": HEIGHT + 0.08975,
        "toolLocalTcpRosM": links["tcp"]["position"],
        "sources": [{"root": str(root.resolve()), "files": verify_source(root.resolve())} for root in args.ros_root],
        "unityCatalogSha256": digest(PROJECT / "Assets/Resources/RobotModels/Models.json"),
        "launchValidation": "AST defaults and Xacro XML; ROS launch libraries not executed",
        "hardwareExercised": False, "rosNodesStarted": False, "deploymentPerformed": False,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Verified {len(args.ros_root)} ROS source trees, six launch defaults per tree, and the Unity 143 mm tool chain")


if __name__ == "__main__":
    main()
