#!/usr/bin/env python3
"""Record passing five-point registration, orbit and bounded adjustment checks."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
from xml.etree import ElementTree


PROJECT = Path(__file__).resolve().parents[2]
REGISTRATION_FIXTURES = {
    "RobotCoverLandmarkTests", "RobotPointRegistrationMathTests", "RobotPointRegistrationWorkflowTests",
    "RobotRegistrationGuideTests", "PointPlacementMenuTests",
}
CONTACT_FIXTURES = {"TargetPoseContextTests", "TargetSelectionGestureTests", "PickPlaceToolContactTests"}


def fingerprint(path):
    return {"path": path.relative_to(PROJECT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, default=PROJECT / "Library/FivePointRegistrationValidation")
    parser.add_argument("--results-file", default="five-point-results.xml")
    parser.add_argument("--render-log", default="five-point-previews.log")
    parser.add_argument("--preview-directory", default="FivePointRegistrationPreviews")
    parser.add_argument("--contact-render-log", default="five-point-previews.log")
    args = parser.parse_args()
    isolated = args.project.resolve()
    library = (PROJECT / "Library").resolve()
    if isolated == library or not isolated.is_relative_to(library):
        raise ValueError("Validation evidence must come from an isolated project below Library")
    results_path = isolated / args.results_file
    result = ElementTree.parse(results_path).getroot()
    if int(result.get("failed", "1")) or int(result.get("passed", "0")) == 0:
        raise ValueError("A passing editor result is required")
    fixture_counts = Counter(case.get("classname") for case in result.iter("test-case")
                             if case.get("classname") in REGISTRATION_FIXTURES and case.get("result") == "Passed")
    if set(fixture_counts) != REGISTRATION_FIXTURES:
        raise ValueError("All five registration fixtures must have passing cases")
    for case in result.iter("test-case"):
        if case.get("classname") in REGISTRATION_FIXTURES and case.get("result") != "Passed":
            raise ValueError("A registration case did not pass: " + str(case.get("fullname")))
    contact_counts = Counter(case.get("classname") for case in result.iter("test-case")
                             if case.get("classname") in CONTACT_FIXTURES and case.get("result") == "Passed")
    if set(contact_counts) != CONTACT_FIXTURES:
        raise ValueError("All target gesture/contact fixtures must have passing cases")
    for case in result.iter("test-case"):
        if case.get("classname") in CONTACT_FIXTURES and case.get("result") != "Passed":
            raise ValueError("A target gesture/contact case did not pass: " + str(case.get("fullname")))
    skipped = [{"name": case.get("fullname"), "reason": case.findtext("reason/message")}
               for case in result.iter("test-case") if case.get("result") == "Skipped"]

    sources = {
        PROJECT / "Assets/Scripts/V3/RobotPointRegistration.cs",
        PROJECT / "Assets/Scripts/V3/URInstantiator.cs",
        PROJECT / "Assets/Scripts/V3/URInputManager.cs",
        PROJECT / "Assets/Scripts/V3/UI/RobotRegistrationGuide.cs",
        PROJECT / "Assets/Scripts/V3/ROS/RobotCoordinateService.cs",
        PROJECT / "Assets/Scripts/V3/ROS/RobotStateService.cs",
        PROJECT / "Assets/Scripts/V3/ROS/JointStateSubscriber.cs",
        PROJECT / "Assets/Resources/RobotModels/Models.json",
        PROJECT / "Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab",
        PROJECT / "Assets/Prefabs/ur16e_kenos/meshes/ur_description/meshes/ur10e/visual/shoulder.dae",
    }
    sources.update((PROJECT / "Assets/Scripts/V3/UI").glob("ToolSelectionMenu*.cs"))
    sources.update((PROJECT / "Assets/Scripts/V3/Frames").glob("*.cs"))
    sources.update(PROJECT / "Assets/Tests/RobotFrames" / (name + ".cs") for name in REGISTRATION_FIXTURES)
    sources.update(PROJECT / "Assets/Tests/RobotFrames" / (name + ".cs") for name in CONTACT_FIXTURES)
    sources.update(PROJECT / "Assets/Scripts/V3" / name for name in (
        "TargetPoseContextMenu.cs", "TargetRotationHandles.cs", "NormalTargetMarker.cs", "ToolPoseVisual.cs"))
    sources.add(PROJECT / "Assets/Scripts/V3/ROS/PickPlaceController.cs")
    sources.update(PROJECT / "Assets/Resources/RobotTargetMarkers" / name for name in (
        "RegistrationGuideTarget.shader", "RegistrationGuideTarget.mat"))
    sources.update(Path(str(path) + ".meta") for path in tuple(sources) if Path(str(path) + ".meta").is_file())
    for source in sources:
        tested = isolated / source.relative_to(PROJECT)
        if source.read_bytes() != tested.read_bytes():
            raise ValueError("A tested copy is stale: " + str(source))

    compiler_path = PROJECT / "Library/FivePointRegistrationValidationSupport/runtime_compile_summary.json"
    compiler = json.loads(compiler_path.read_text(encoding="utf-8-sig"))
    if compiler["errorCount"] or compiler["exitCode"]:
        raise ValueError("The complete runtime must compile")
    for entry in compiler["validatedSources"]:
        if fingerprint(PROJECT / entry["path"])["sha256"] != entry["sha256"]:
            raise ValueError("Runtime changed after compilation: " + entry["path"])

    preview_directory = isolated / args.preview_directory
    previews = sorted(preview_directory.rglob("*.png"))
    expected_previews = ["placement.png", "failure.png", "adjustment.png"] + [f"Guide/step-{index}.png" for index in range(1, 6)]
    for name in expected_previews:
        if not (preview_directory / name).is_file():
            raise ValueError("Required registration preview is missing: " + name)
    render_log_path = isolated / args.render_log
    render_log = render_log_path.read_text(encoding="utf-8", errors="replace")
    if "text overflow" in render_log.lower() or "TextOverflow" in render_log:
        raise ValueError("Rendered text overflow must be resolved")
    if "shader error in" in render_log.lower():
        raise ValueError("A rendered shader failed compilation")
    marker_pixels = {int(step): int(count) for step, count in re.findall(
        r"RegistrationGuidePreview: step=(\d+), orangeTargetPixels=(\d+)", render_log)}
    if set(marker_pixels) != set(range(1, 6)) or any(count <= 8 for count in marker_pixels.values()):
        raise ValueError("All five contact markers must render visible orange pixels")
    contact_preview = isolated / "normal-contact-context.png"
    contact_log = isolated / args.contact_render_log
    if not contact_preview.is_file() or "ToolContactPreview:" not in contact_log.read_text(encoding="utf-8", errors="replace"):
        raise ValueError("The actual ground-locked target context preview is required")

    report = {
        "schemaVersion": 1, "completedUtc": result.get("end-time"), "unityVersion": "6000.0.84f1",
        "scope": "Five blue registration covers from base to tool, bounded XYZ correction, target click/drag gestures and ground-locked MTC contact normals",
        "editorTests": {key: int(result.get(key, "0")) for key in ("total", "passed", "failed", "skipped")},
        "skippedCases": skipped,
        "registrationTests": {"total": sum(fixture_counts.values()), "fixtures": dict(sorted(fixture_counts.items()))},
        "targetContactTests": {"total": sum(contact_counts.values()), "fixtures": dict(sorted(contact_counts.items()))},
        "testResults": fingerprint(results_path),
        "landmarks": {"orderedNames": ["Base cover", "Shoulder", "Elbow", "Wrist 2", "Wrist 3"],
                      "firstOwningLink": "ur_shoulder_link", "firstSurface": "Lowest blue pan-joint cover, upstream of the shoulder-lift cover",
                      "firstPointInVisualRosM": [-0.000003, 0.000008, 0.087942],
                      "provenance": "CAD-derived blue surface on shoulder.dae; checked against the URBlue material and calibrated moving-link FK"},
        "miniature": {"turnPeriodSeconds": 10, "checks": "Deterministic orbit, stable geometry/centering, all five highlighted contacts and unchanged live/locked joint inputs"},
        "positionCorrection": {"axes": "ROS robot-local XYZ", "maximumTotalOffsetMeters": .02,
                               "bound": "Euclidean displacement from the original fit across repeated adjustments",
                               "orientationChanged": False, "fitResidualsRecomputedAsAccuracy": False},
        "runtimeCompile": {"helperSources": compiler["helperSourceCount"], "mainSources": compiler["mainSourceCount"],
                           "errors": 0, "warnings": compiler["warnings"], "evidence": fingerprint(compiler_path)},
        "visualQa": {"log": fingerprint(render_log_path), "previews": [fingerprint(path) for path in previews],
                     "groundLockContextPreview": fingerprint(contact_preview), "groundLockContextLog": fingerprint(contact_log),
                     "markerPixelChecks": "All five current contact markers remain visible using the bundled XR-aware overlay shader/material",
                     "orangeMarkerPixelsByStep": marker_pixels,
                     "shaderValidation": "Graphics-enabled mobile URP D3D11 render; no shader compilation errors; native headset rendering not exercised"},
        "sources": [fingerprint(path) for path in sorted(sources)],
        "tooling": [fingerprint(PROJECT / "Tools/RobotFrames" / name) for name in (
            "prepare_unity_validation.py", "prepare_point_registration_validation.py",
            "prepare_fixture_tiling_validation.py", "prepare_scene_workflow_validation.py",
            "prepare_five_point_registration_validation.py", "record_five_point_registration_validation.py")],
        "hardwareExercised": False, "editorRosTransport": False, "headsetTested": False,
        "robotMotionRequested": False, "deploymentPerformed": False,
        "limits": ["The lowest blue cover contact is derived from CAD and still needs physical verification.",
                   "Residuals measure agreement between captured points, not absolute physical alignment accuracy.",
                   "A manual overlay correction does not recalibrate the controller's physical contact tip.",
                   "Native headset tracking, physical contact capture and live ROS execution are not exercised."],
    }
    output = PROJECT / "Tools/RobotFrames/five_point_registration_validation_summary.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {report['editorTests']['passed']} passing editor cases, including {sum(fixture_counts.values())} registration cases")


if __name__ == "__main__":
    main()
