#!/usr/bin/env python3
"""Record verified fixture-tiling test, schema, compiler and menu evidence."""

from collections import Counter
import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree


PROJECT = Path(__file__).resolve().parents[2]
VALIDATION = PROJECT / "Library/FixtureTilingValidation"
SUPPORT = PROJECT / "Library/FixtureTilingValidationSupport"


def fingerprint(path):
    return {"path": path.relative_to(PROJECT).as_posix(),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    results_path = VALIDATION / "fixture-final-results.xml"
    result = ElementTree.parse(results_path).getroot()
    if int(result.get("failed", "0")) or result.get("result") != "Passed":
        raise ValueError("The final complete editor suite must pass")
    compiler_path = SUPPORT / "runtime_compile_summary.json"
    compiler = json.loads(compiler_path.read_text(encoding="utf-8-sig"))
    if compiler["errorCount"] or compiler["exitCode"]:
        raise ValueError("The runtime compilation must pass")
    for entry in compiler["validatedSources"]:
        if fingerprint(PROJECT / entry["path"])["sha256"] != entry["sha256"]:
            raise ValueError("Runtime source changed after compilation: " + entry["path"])

    manifest_path = PROJECT / "Tools/RobotFrames/RosMessageSchemas/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    for entry in manifest["messages"]:
        schema = manifest_path.parent / entry["package"] / "msg" / (entry["name"] + ".msg")
        if fingerprint(schema)["sha256"] != entry["sha256"]:
            raise ValueError("A pinned ROS schema changed")

    sources = sorted(set(
        path for folder in ("Assets/Scripts/V3/ROS", "Assets/Tests/RobotFrames")
        for path in (PROJECT / folder).glob("Fixture*.cs")
    ) | {PROJECT / relative for relative in (
        "Assets/Scripts/V3/URInputManager.cs", "Assets/Scripts/V3/UI/ToolSelectionMenu.cs",
        "Assets/Scripts/V3/UI/ToolSelectionMenu.Fixture.cs", "Assets/Scripts/V3/UI/MTCPlanUIManager.cs",
    )})
    for path in sources:
        copied = VALIDATION / path.relative_to(PROJECT)
        if copied.read_bytes() != path.read_bytes():
            raise ValueError("Tested isolated copy is stale: " + str(path))
    fixture_counts = Counter(case.get("classname") for case in result.iter("test-case")
                             if case.get("classname", "").startswith("Fixture"))
    menu_log = VALIDATION / "fixture-menu-preview.log"
    menu_text = menu_log.read_text(encoding="utf-8", errors="replace")
    if "FixturePreviewTextOverflow" in menu_text or "return code 0" not in menu_text:
        raise ValueError("Menu export must complete without text overflow")
    previews = sorted((VALIDATION / "FixtureTilingPreviews").glob("*.png"))
    if len(previews) != 10:
        raise ValueError("Expected nine rendered menu views and one contact sheet")

    report = {
        "schemaVersion": 1, "completedUtc": result.get("end-time"),
        "scope": "Square-tile fixture setup, identified complete-batch review and single-use execution admission",
        "unityVersion": "6000.0.84f1",
        "editorTests": {key: int(result.get(key)) for key in ("total", "passed", "failed", "skipped")},
        "newFixtureTests": {"total": sum(fixture_counts.values()), "fixtures": dict(sorted(fixture_counts.items()))},
        "testResults": fingerprint(results_path),
        "runtimeCompile": {"compiler": compiler["compiler"], "helperSources": compiler["helperSourceCount"],
                           "mainSources": compiler["mainSourceCount"], "errors": 0,
                           "warnings": compiler["warnings"], "evidence": fingerprint(compiler_path)},
        "schemas": {"manifest": fingerprint(manifest_path), "closureMessages": len(manifest["messages"]),
                    "generatedMessages": sum(bool(entry["generate"]) for entry in manifest["messages"]),
                    "fixtureBuildImage": manifest["fixtureBuildImage"],
                    "installedBackendComparison": manifest["fixtureBuildComparison"],
                    "cdrFixture": fingerprint(PROJECT / "Assets/Tests/RobotFrames/Resources/FixtureSolutionCdr.json"),
                    "cdrVerification": "Independent ROS Python CDR with polygon, texture, UV and mesh-file extensions; Unity parses fields and reproduces all meaningful bytes with only schema-derived padding normalized",
                    "privateNestedRegistrations": "No global registry hooks; compatible standard messages use connector types"},
        "menuVisualQa": {"renderedViews": 9, "textOverflowWarnings": 0, "log": fingerprint(menu_log),
                         "previews": [fingerprint(path) for path in previews]},
        "sources": [fingerprint(path) for path in sources],
        "rosMessageSources": [fingerprint(path) for path in sorted((PROJECT / "Assets/RosMessages").rglob("*.cs"))],
        "tooling": [fingerprint(PROJECT / "Tools/RobotFrames" / name) for name in (
            "prepare_fixture_tiling_validation.py", "generate_fixture_preview_messages.ps1",
            "generate_fixture_solution_cdr.py", "record_fixture_tiling_validation.py")],
        "editorRosTransport": False, "hardwareExercised": False, "robotMotionRequested": False,
        "deploymentPerformed": False,
        "testIsolation": "Actual controllers, menu, scene reviewer, rig and models in a separate project; ROS factory throws and publication overrides record requests without transport",
    }
    output = PROJECT / "Tools/RobotFrames/fixture_tiling_validation_summary.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {report['editorTests']['passed']} passing checks, including {sum(fixture_counts.values())} fixture checks")


if __name__ == "__main__":
    main()
