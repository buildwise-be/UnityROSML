#!/usr/bin/env python3
"""Record passing point-registration tests only when current sources match the tested copies."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=PROJECT / "Library/PointRegistrationValidation")
    args = parser.parse_args()
    isolated = args.project.resolve()
    results = isolated / "point-results.xml"
    run = ET.parse(results).getroot()
    if run.get("result") != "Passed" or int(run.get("failed", "1")) != 0 or int(run.get("passed", "0")) == 0:
        raise ValueError("A complete passing Unity test result is required")
    paths = [
        "Assets/Scripts/V3/RobotPointRegistration.cs",
        "Assets/Scripts/V3/URInstantiator.cs",
        "Assets/Scripts/V3/URInputManager.cs",
        "Assets/Scripts/V3/ROS/JointStateSubscriber.cs",
        "Assets/Scripts/V3/ROS/RobotCoordinateService.cs",
        "Assets/Scripts/V3/ROS/RobotStateService.cs",
        "Assets/Scripts/V3/ROS/RosRobotRig.cs",
        "Assets/Scripts/V3/UI/ToolSelectionMenu.cs",
        "Assets/Resources/RobotModels/Models.json",
        "Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab",
    ]
    for folder in ("Assets/Scripts/V3/Frames", "Assets/Tests/RobotFrames"):
        paths.extend(path.relative_to(PROJECT).as_posix() for path in (PROJECT / folder).glob("*.cs")
                     if (isolated / path.relative_to(PROJECT)).is_file())
    validated = []
    for relative in sorted(set(paths)):
        actual, tested = PROJECT / relative, isolated / relative
        if digest(actual) != digest(tested):
            raise ValueError("Source differs from its tested copy: " + relative)
        validated.append({"path": relative, "sha256": digest(actual)})
    summary = {
        "schemaVersion": 1,
        "completedUtc": datetime.now(timezone.utc).isoformat(),
        "unityVersion": "6000.0.84f1",
        "unityTests": {key: int(run.get(key, "0")) for key in ("total", "passed", "failed", "skipped")},
        "fixtures": sorted({case.get("classname") for case in run.iter("test-case")}),
        "testResults": {"path": str(results), "sha256": digest(results)},
        "validatedSources": validated,
        "hardwareExercised": False,
        "networkExercised": False,
        "activeProjectOpened": False,
        "boundaries": [
            "Actual placement/subscriber/rig/input/menu runtime sources, native articulation physics and CAD mesh contacts.",
            "QR hardware is an inert synthetic-pose stub. ROSService throws before any connection or subscription.",
            "No headset tracking, physical tip measurement, physical cover measurement or real robot motion was tested.",
            "Fit residual measures agreement of four contacts, not absolute physical alignment accuracy.",
        ],
    }
    preview = isolated / "point-placement-menu.png"
    if preview.is_file():
        summary["menuPreview"] = {"path": str(preview), "sha256": digest(preview)}
    output = PROJECT / "Tools/RobotFrames/point_registration_validation_summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {summary['unityTests']['passed']} passing Unity tests with {len(validated)} matching source hashes")


if __name__ == "__main__":
    main()
