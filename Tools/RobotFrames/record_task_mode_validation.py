#!/usr/bin/env python3
"""Record focused task-mode tests and graphics-enabled previews, with source hashes."""

import argparse
from collections import Counter
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
    parser.add_argument("--results", default="task-mode-results.xml")
    parser.add_argument("--log", default="task-mode-editor.log")
    parser.add_argument("--preview-log", default="task-mode-preview-editor.log")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("task_mode_validation_summary.json"))
    parser.add_argument("--follow-validation", action="store_true", help="Also require and record the actual world-menu follow tests.")
    args = parser.parse_args()
    isolated = args.validation_project.resolve()
    result_file = isolated / args.results
    log_file = isolated / args.log
    render_log = isolated / args.preview_log
    results = ET.parse(result_file).getroot()
    cases = list(results.iter("test-case"))
    if not cases or results.get("failed") != "0" or any(case.get("result") != "Passed" for case in cases):
        raise ValueError("Focused tests must all pass")
    names = Counter(case.get("classname") for case in cases)
    if names["TaskModeSwitchTests"] < 18:
        raise ValueError("Expected all focused task-mode cases")
    if args.follow_validation and names["MenuFollowTests"] < 20:
        raise ValueError("Expected menu-follow behavior cases")
    sources = []
    for copied in (isolated / "Assets").rglob("*.cs"):
        relative = copied.relative_to(isolated)
        original = project / relative
        if original.is_file():
            if digest(copied) != digest(original):
                raise ValueError(f"Validated source differs from current workspace: {relative}")
            sources.append({"path": relative.as_posix(), "sha256": digest(original), "matchesValidatedCopy": True})
    previews = []
    for mode, name in (("Exact TCP", "task"), ("MTC object-surface pick/place", "pick-place"), ("XR alignment", "alignment")):
        path = isolated / f"calibration-menu-{name}.png"
        previews.append({"mode": mode, "path": str(path), "sha256": digest(path), "sizeBytes": path.stat().st_size})
    summary = {
        "schemaVersion": 1, "completedUtc": results.get("end-time"), "unityVersion": "6000.0.84f1",
        "success": True,
        "unityTests": {"total": len(cases), "passed": len(cases), "failed": 0, "byFixture": dict(sorted(names.items()))},
        "scope": [
            "Actual URInputManager, PickPlaceController, MTCPlansManager, TcpPlanningController, TcpTargetPreview and shared trajectory visualizer",
            "Retained targets, inactive marker visibility/capture/export, interaction cancellation, task selection and planner routing",
            "TCP/MTC review invalidation, legacy initial-progress/failure messages, late-result rejection, preview timeout and single-use execution authorization",
            "Actual unchanged robot prefab and ROS model produce native trajectory clones for successful MTC preview tests",
            "Tool matching, calibration command gates, legacy feedback and graphics-enabled shared menu previews"
        ],
        "isolatedWorkflowStubs": ["ROSService factory throws on access", "RemoteExecutionController pose-source binding is inert", "QRDetector/URInstantiator hardware APIs are inert; independently validated previously"],
        "fixtureNotes": ["Marker roots are seeded to test mode visibility and pose retention without requiring live XR tracking.", "No controller Start method that opens ROS transport is invoked. Offline publish failures exercise the real legacy controller catch path.", "Preview screenshots show the actual runtime world canvas with representative offline status text."],
        "hardwareExercised": False, "networkExercised": False, "activeUnityEditorModified": False,
        "limitations": ["This does not exercise ROS service/topic transport, real MTC planning, headset controller tracking or robot motion.", "Legacy MTC status/trajectory messages have no request identifier; external concurrent clients cannot be uniquely correlated."],
        "resultXml": {"path": str(result_file), "sha256": digest(result_file)},
        "editorLog": {"path": str(log_file), "sha256": digest(log_file)},
        "previewEditorLog": {"path": str(render_log), "sha256": digest(render_log)},
        "previews": previews, "validatedSources": sorted(sources, key=lambda item: item["path"]),
        "validationWorkflowSources": [{"path": path.relative_to(project).as_posix(), "sha256": digest(path)} for path in (
            project / "Tools/RobotFrames/prepare_unity_validation.py", Path(__file__).resolve())],
    }
    if args.follow_validation:
        summary["scope"].append("Actual world-menu dead zones, smooth bounded following around the viewer, user-facing rotation, hover/calibration/target-drag locks, focus/pause cleanup and gentle resume")
        menu_source = (project / "Assets/Scripts/V3/UI/ToolSelectionMenu.cs").read_text(encoding="utf-8")
        summary["menuFollowDefaults"] = {name: float(value) for name, value in re.findall(r"private float (_follow\w+) = (-?[0-9.]+)f;", menu_source)}
        measured = re.search(r"MenuFollowValidation: frame-rate-position-difference-metres=([0-9.eE+-]+)", log_file.read_text(encoding="utf-8", errors="replace"))
        if measured is None:
            raise ValueError("Expected the measured 30/120 Hz follow comparison")
        summary["frameRateComparison"] = {"framesPerSecond": [30, 120], "durationSeconds": 8,
            "finalPositionDifferenceMetres": float(measured.group(1)), "toleranceMetres": 0.025}
        turn = re.search(r"MenuFollowValidation: turn-clearance-minimum-metres=([0-9.eE+-]+)", log_file.read_text(encoding="utf-8", errors="replace"))
        if turn is None:
            raise ValueError("Expected the measured 180-degree turn clearance")
        summary["turnAroundClearance"] = {"headTurnDegrees": 180, "durationSeconds": 6,
            "minimumHorizontalDistanceMetres": float(turn.group(1)), "requiredMinimumMetres": 0.85}
    output = args.output
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {output}: {len(cases)} focused tests passed; three actual menu previews recorded")


if __name__ == "__main__":
    main()
