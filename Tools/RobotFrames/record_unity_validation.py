#!/usr/bin/env python3
"""Persist a compact measured summary and source hashes from isolated Unity tests."""

import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    project = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validation-project", type=Path, default=project / "Library/RosRigValidation")
    parser.add_argument("--results", default="verified-test-results.xml")
    parser.add_argument("--log", default="verified-editor.log")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("unity_validation_summary.json"))
    args = parser.parse_args()
    isolated = args.validation_project
    result_file, log_file = isolated / args.results, isolated / args.log
    results = ET.parse(result_file).getroot()
    measurements = []
    pattern = re.compile(r"^RosRigValidation (main|transparent)/(\w+): maxPositionMetres=(.*?), maxBasisVectorError=(.*)$", re.MULTILINE)
    for prefab, tool, position, basis in pattern.findall(log_file.read_text(encoding="utf-8", errors="replace")):
        measurements.append({"prefab": prefab, "tool": tool,
                             "maximumPositionErrorMetres": float(position.strip().replace(",", ".")),
                             "maximumBasisVectorError": float(basis.strip().replace(",", "."))})
    if results.get("failed") != "0" or len(measurements) != 6:
        raise ValueError("Expected a passing test run and six measured prefab/tool combinations")
    sources = [project / relative for relative in (
        "Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab", "Assets/Prefabs/ur16e_kenos/ur16e_kenos_transparent.prefab",
        "Assets/Scripts/V3/ROS/RosRobotRig.cs", "Assets/Scripts/V3/ROS/ToolSelectionController.cs",
        "Assets/Scripts/V3/ROS/RobotDescriptionMessages.cs", "Assets/Scripts/V3/UI/ToolSelectionMenu.cs",
        "Assets/Scripts/V3/ROS/TcpPlanningController.cs", "Assets/Scripts/V3/TcpTargetPreview.cs",
        "Assets/Scripts/V3/URInputManager.cs", "Assets/Scripts/V3/TargetRotationHandles.cs",
        "Assets/Scripts/V3/XRTrackingSpaceResolver.cs", "Assets/Scripts/V3/UI/MTCPlanUIManager.cs",
        "Assets/Scripts/V3/ROS/RobotCoordinateService.cs", "Assets/Scripts/V3/ROS/RobotStateService.cs",
        "Assets/Scripts/V3/ROS/MTCPlansManager.cs", "Assets/Scripts/V3/ROS/PickPlaceController.cs",
        "Assets/Scripts/V3/ROS/UR16eTrajectoryVisualizer.cs",
        "Assets/Samples/Magic Leap SDK/2.6.0/ML Rig & OpenXR Input/MagicLeapOpenXRInput.cs",
    )]
    for folder, extension in (("Assets/Scripts/V3/Frames", "*.cs"), ("Assets/Tests/RobotFrames", "*.cs"),
                              ("Assets/Tests/RobotFrames/Fixtures", "*"), ("Assets/Resources/RobotModels", "*")):
        sources.extend(path for path in (project / folder).glob(extension) if path.is_file() and path.suffix != ".meta"
                       and path.name not in ("QrCalibrationTests.cs", "XrAlignmentIntegrationTests.cs"))
    sources.extend((project / "Assets/RosMessages").rglob("*.cs"))
    hashes = []
    for source in sorted(set(sources)):
        relative = source.relative_to(project)
        copied = isolated / relative
        if not copied.is_file() or digest(source) != digest(copied):
            raise ValueError(f"Current source differs from the validated copy: {relative}")
        hashes.append({"path": relative.as_posix(), "sha256": digest(source), "matchesValidatedCopy": True})
    summary = {
        "schemaVersion": 1, "completedUtc": results.get("end-time"), "unityVersion": "6000.0.84f1",
        "unityTests": {"total": int(results.get("total")), "passed": int(results.get("passed")), "failed": 0,
                       "scope": "Actual serialized main/transparent prefabs and native articulation physics; CAD geometry/materials; model/parser math; TCP callbacks, input routing and status reset; shared tool/task/alignment world menu; calibration command gates and profile persistence; installed ROS2 CDR bytes"},
        "postPhysicsGoldenLinkPoses": 1806,
        "maximumPositionErrorMetres": max(item["maximumPositionErrorMetres"] for item in measurements),
        "maximumBasisVectorError": max(item["maximumBasisVectorError"] for item in measurements),
        "physicsPositionToleranceMetres": 0.00002, "physicsBasisVectorTolerance": 0.00002,
        "measurements": measurements,
        "rosMessageFixtures": "Actual installed rcl_interfaces + rclpy serialization, without rclpy.init; unspecified CDR alignment padding masked using schema-derived offsets",
        "isolatedWorkflowStubs": ["ROSService (throws if requested)", "RemoteExecutionController (inert pose-source binding only)",
                                  "QRDetector and URInstantiator (hardware/placement tested separately)"],
        "delegatedTests": ["QrCalibrationTests and XrAlignmentIntegrationTests: independent actual SDK/placement audit"],
        "hardwareExercised": False, "networkExercised": False, "activeUnityEditorModified": False,
        "limitations": ["ROS planning, service transport, hardware tracking, physical tool calibration and robot operation are not exercised.",
                        "Read-only GetParameters metadata handling is tested offline; the actual ROS service callback transport is not exercised.",
                        "Matching calibrated ROS/Unity model definitions does not prove that provisional tool dimensions match the physical installation."],
        "resultXmlSha256": digest(result_file), "editorLogSha256": digest(log_file), "validatedSources": hashes,
    }
    output = args.output
    output.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Saved {output}: {summary['unityTests']['passed']} tests passed, max {summary['maximumPositionErrorMetres'] * 1e6:.3f} micrometres")


if __name__ == "__main__":
    main()
