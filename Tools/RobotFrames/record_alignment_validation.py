#!/usr/bin/env python3
"""Record actual-component XR calibration checks and exact validated source hashes."""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation-project", type=Path, required=True)
    parser.add_argument("--results", default="alignment-results.xml")
    parser.add_argument("--log", default="alignment-editor.log")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("alignment_validation_summary.json"))
    args = parser.parse_args()
    isolated = args.validation_project.resolve()
    results_file = isolated / args.results
    log_file = isolated / args.log
    results = ET.parse(results_file).getroot()
    if results.get("failed") != "0" or results.get("skipped") != "0":
        raise ValueError("Expected a clean actual-component calibration test run")
    groups = {}
    for case in results.iter("test-case"):
        name = case.get("classname")
        group = groups.setdefault(name, {"passed": 0, "failed": 0})
        group["passed" if case.get("result") == "Passed" else "failed"] += 1
    if groups.get("XrAlignmentIntegrationTests", {}).get("passed", 0) < 10:
        raise ValueError("Expected all ten real-component alignment integration scenarios")

    relatives = ["Assets/Scripts/V3/URInstantiator.cs", "Assets/Scripts/V3/ROS/RosRobotRig.cs",
                 "Assets/Scripts/V3/ROS/RobotCoordinateService.cs", "Assets/Scripts/V3/ROS/RobotStateService.cs",
                 "Assets/Scripts/V3/XRTrackingSpaceResolver.cs", "Assets/Tests/RobotFrames/CalibrationProfileTests.cs",
                 "Assets/Tests/RobotFrames/QrCalibrationTests.cs", "Assets/Tests/RobotFrames/XrAlignmentIntegrationTests.cs",
                 "Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab"]
    for folder in ("Assets/Scripts/V3/Frames", "Assets/Scripts/V3/QR"):
        relatives.extend(path.relative_to(PROJECT).as_posix() for path in (PROJECT / folder).glob("*.cs"))
    hashes = []
    for relative in sorted(set(relatives)):
        source, copied = PROJECT / relative, isolated / relative
        if not copied.is_file() or digest(source) != digest(copied):
            raise ValueError(f"Current source differs from the validated actual component: {relative}")
        hashes.append({"path": relative, "sha256": digest(source), "matchesValidatedCopy": True})
    data = {
        "schemaVersion": 1,
        "completedUtc": results.get("end-time"),
        "unityVersion": "6000.0.84f1",
        "magicLeapSdkVersion": "2.6.0",
        "unityTests": {"total": int(results.get("total")), "passed": int(results.get("passed")), "failed": 0, "skipped": 0},
        "testGroups": groups,
        "scope": "Actual QR, URInstantiator, native articulation, coordinate service and app-local calibration persistence components.",
        "testedBehavior": [
            "Draft world-space translations and full base-pivot rotations preserve live joint values.",
            "Calibration editing immediately blocks coordinate conversion and survives concurrent initialization attempts.",
            "Save writes QR-relative pose, which reloads under different XR origins and marker poses after component restart.",
            "A distinct QR payload cannot inherit another marker's saved calibration.",
            "Reset remains an unsaved draft; Cancel restores the previous saved pose.",
            "Rescan discards an edit and leaves commands blocked until confirmed placement.",
            "Saved alignment changes real TCP pose conversion to the corrected base frame and advances calibration revision.",
            "Draft and cancel remain QR-relative when the XR tracking origin moves during editing.",
            "Failed storage writes retain the visible draft and keep commands blocked for retry or cancel.",
            "Beginning immediately after XR origin relocation uses authoritative calibration instead of stale native physics position."
        ],
        "positionToleranceMetres": 0.00002,
        "basisVectorTolerance": 0.00002,
        "profileTestsAlsoRunInWorkflowSuite": True,
        "isolatedWorkflowStubs": ["ROSService throws if requested", "ToolSelectionController", "ToolSelectionMenu", "command/preview invalidation surfaces", "TargetRotationHandles cancellation"],
        "profileStorage": "Only test-owned GUID temporary directories; the real application's persistent calibration store is not modified.",
        "hardwareExercised": False,
        "networkExercised": False,
        "limitations": ["Headset interaction, marker tracking, physical overlay accuracy and real robot operation are not exercised.",
                        "The actual menu and command-gate regressions are verified separately by the workflow suite.",
                        "File replacement is exercised on Windows; Android profile replacement is compiled but not run on an ML2 device."],
        "evidence": {"results": str(results_file), "log": str(log_file), "resultSha256": digest(results_file), "logSha256": digest(log_file)},
        "validatedSources": hashes
    }
    args.output.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Saved {args.output}: {data['unityTests']['passed']} actual-component calibration tests passed")


if __name__ == "__main__":
    main()
