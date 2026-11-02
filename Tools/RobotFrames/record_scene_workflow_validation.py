#!/usr/bin/env python3
"""Record scene separation, routing, lifecycle and visual validation evidence."""

import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree


PROJECT = Path(__file__).resolve().parents[2]
VALIDATION = PROJECT / "Library/FixtureTilingValidation"
SUPPORT = PROJECT / "Library/SceneWorkflowValidationSupport"


def fingerprint(path):
    return {"path": path.relative_to(PROJECT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def test_result(path):
    root = ElementTree.parse(path).getroot()
    if int(root.get("failed", "0")):
        raise ValueError("Test failures remain: " + str(path))
    cases = {case.get("fullname"): case.get("result") for case in root.iter("test-case")}
    return {"total": int(root.get("total")), "passed": int(root.get("passed")),
            "failed": int(root.get("failed")), "skipped": int(root.get("skipped")),
            "completedUtc": root.get("end-time"), "evidence": fingerprint(path)}, cases


def main():
    baseline, baseline_cases = test_result(VALIDATION / "scene-baseline-results.xml")
    actual, actual_cases = test_result(VALIDATION / "scene-real-service-results.xml")
    if baseline["skipped"] != 1 or actual["skipped"] != 0:
        raise ValueError("Expected only the baseline's intentional actual-service test skip")
    successful = {name for name, result in baseline_cases.items() if result == "Passed"}
    successful.update(name for name, result in actual_cases.items() if result == "Passed")
    if set(baseline_cases) - successful:
        raise ValueError("A baseline test is not covered by a successful run")

    compiler_path = SUPPORT / "runtime_compile_summary.json"
    compiler = json.loads(compiler_path.read_text(encoding="utf-8-sig"))
    if compiler["errorCount"] or compiler["exitCode"]:
        raise ValueError("Runtime compilation must pass")
    for entry in compiler["validatedSources"]:
        if fingerprint(PROJECT / entry["path"])["sha256"] != entry["sha256"]:
            raise ValueError("Runtime source changed after compilation: " + entry["path"])

    preparation_path = VALIDATION / "preparation-report.json"
    preparation = json.loads(preparation_path.read_text())
    scene_harness = preparation["sceneWorkflowHarness"]
    scene_manifest = scene_harness["serializedSources"]
    for entry in scene_manifest["scenes"]:
        if fingerprint(PROJECT / entry["path"])["sha256"] != entry["sha256"]:
            raise ValueError("Scene changed after snapshots were prepared: " + entry["path"])
    build_path = PROJECT / "ProjectSettings/EditorBuildSettings.asset"
    if fingerprint(build_path)["sha256"] != scene_manifest["buildSettings"]["sha256"]:
        raise ValueError("Build settings changed after snapshot preparation")

    sources = [PROJECT / path for path in (
        "Assets/Scripts/V3/RobotAppNavigation.cs", "Assets/Scripts/V3/URInputManager.cs",
        "Assets/Scripts/V3/UI/RobotWelcomeMenu.cs", "Assets/Scripts/V3/UI/ToolSelectionMenu.cs",
        "Assets/Scripts/V3/UI/ToolSelectionMenu.Fixture.cs", "Assets/Scripts/V3/UI/ToolSelectionMenu.Navigation.cs",
        "Assets/Scripts/V3/ROS/ROSService.cs", "Assets/Scripts/V3/ROS/RobotStateService.cs",
        "Assets/Scripts/V3/ROS/RobotCoordinateService.cs", "Assets/Scripts/V3/ROS/MTCPlansManager.cs",
        "Assets/Tests/RobotFrames/RobotAppSceneTests.cs", "Assets/Tests/RobotFrames/WorkflowMenuTests.cs",
        "Assets/Tests/RobotFrames/WorkflowServiceLifecycleTests.cs", "Assets/Tests/RobotFrames/FixtureTilingMenuTests.cs",
    )]
    for source in sources:
        if source.read_bytes() != (VALIDATION / source.relative_to(PROJECT)).read_bytes():
            raise ValueError("Tested source is stale: " + str(source))
    previews = sorted((VALIDATION / "WorkflowMenuPreviews").rglob("*.png"))
    if not (VALIDATION / "WorkflowMenuPreviews/Welcome.png").is_file() or len(previews) < 11:
        raise ValueError("Welcome and both dedicated workflow menu previews are required")
    visual_log = VALIDATION / "scene-baseline-tests.log"
    log = visual_log.read_text(encoding="utf-8", errors="replace")
    if "FixturePreviewTextOverflow" in log or "Welcome preview text overflow" in log:
        raise ValueError("A rendered menu has overflowing text")
    if "RobotWelcomeMenuPreview:" not in log or log.count("FixtureTilingMenuPreviews:") < 2:
        raise ValueError("All three workflow render exports must complete")

    report = {
        "schemaVersion": 1, "completedUtc": actual["completedUtc"], "unityVersion": "6000.0.84f1",
        "scope": "Welcome scene and physically separate simple/repetitive scenes with guarded navigation and scene-safe robot/ROS lifecycle",
        "baselineThrowingService": baseline,
        "actualServiceFocused": actual,
        "uniqueSuccessfulTestCases": len(successful),
        "intentionalBaselineSkip": "The actual ROSService reuse/recovery case runs successfully in the focused real-service harness instead of the baseline throwing-factory harness.",
        "runtimeCompile": {"helperSources": compiler["helperSourceCount"], "mainSources": compiler["mainSourceCount"],
                           "errors": 0, "warnings": compiler["warnings"], "evidence": fingerprint(compiler_path)},
        "sceneBuildOrder": scene_manifest["scenes"], "buildSettings": fingerprint(build_path),
        "serializedChecks": ["Exactly three enabled scenes; Welcome first", "Unique document IDs and resolved local references",
                             "Welcome contains no robot task controllers or ROS service", "Simple and repetitive scope values and controller wiring",
                             "Repetitive physically omits legacy TCP/pick-place controllers", "One XR rig per scene, no duplicate auto-connecting ROS prefab",
                             "Task services retain the configured 10.200.101.144:10000 initial endpoint"],
        "navigationChecks": ["Known destinations only", "Old alignment/readiness reset before loading", "Duplicate clicks blocked during transition",
                             "Pending/unknown robot commands block navigation before reset", "Loader failure releases navigation lock"],
        "visualQa": {"renderedViews": sum("contact-sheet" not in path.name for path in previews),
                     "textOverflowWarnings": 0, "humanInspection": "Root and UI agent inspected Welcome, Simple main and all repetitive pages",
                     "log": fingerprint(visual_log), "previews": [fingerprint(path) for path in previews]},
        "sources": [fingerprint(path) for path in sources],
        "tooling": [fingerprint(PROJECT / "Tools/RobotFrames" / name) for name in (
            "prepare_unity_validation.py", "prepare_scene_workflow_validation.py", "record_scene_workflow_validation.py")],
        "transport": "Actual ROSService and connector are tested disabled, with ConnectOnStart false; no transport Start/Connect or ROS messages are sent",
        "realServicePersistenceTest": "One UnityTest enters PlayMode in the isolated empty test scene, verifies DontDestroyOnLoad membership of service/connector/replacement, destroys them before ExitPlayMode, and starts no transport",
        "nativeXrSceneLoaded": False, "headsetTested": False, "liveRosTested": False, "robotMotionRequested": False,
        "deploymentPerformed": False,
        "limits": "Serialized scenes are verified through hash-matched text snapshots; scene loading uses the actual guarded router with an injected loader. Native XR scene startup and headset transitions still require device validation.",
    }
    output = PROJECT / "Tools/RobotFrames/scene_workflow_validation_summary.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {len(successful)} unique passing cases across baseline and actual-service runs")


if __name__ == "__main__":
    main()
