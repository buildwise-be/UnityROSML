#!/usr/bin/env python3
"""Prepare actual item/speed pickup, surface capture, guide and wire-contract checks.

This isolated Unity project uses the real source closure and cached connector
assemblies. Tests never call transport Start/Connect or create robot commands;
surface contacts and ROS capability/CDR messages are supplied by the fixtures.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import prepare_force_validation as force
import prepare_unity_validation as common


PROJECT = common.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/PickSurfaceValidation"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    destination = parser.parse_args().destination.resolve()
    library = (PROJECT / "Library").resolve()
    if destination == library or not destination.is_relative_to(library):
        raise ValueError("Pick-surface validation requires a dedicated project below Library")
    arguments = sys.argv[:]
    try:
        sys.argv = [sys.argv[0], "--destination", str(destination)]
        force.main()
    finally:
        sys.argv = arguments
    runtime = sorted({PROJECT / path for path in (
        "Assets/Scripts/V3/ROS/PickPlaceController.cs",
        "Assets/Scripts/V3/ROS/PickSurfaceCapture.cs",
        "Assets/Scripts/V3/UI/PickSurfaceGuide.cs",
        "Assets/Scripts/V3/UI/ToolSelectionMenu.PickSettings.cs",
        "Assets/Scripts/V3/URInputManager.cs",
        "Assets/Scripts/V3/Frames/PickSurfaceGeometry.cs",
        "Assets/RosMessages/Ur16eInterfaces/msg/PickPlaceTaskMsg.cs",
        "Assets/Tests/RobotFrames/PickSurfaceWorkflowTests.cs",
        "Assets/Tests/RobotFrames/PickSurfaceGeometryTests.cs",
        "Assets/Tests/RobotFrames/Resources/PickPlaceTaskCdr.json",
    )})
    for source in runtime:
        common.copy_file(source)
    report_path = destination / "preparation-report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["pickSurfaceHarness"] = {
        "actualSources": [{"path": source.relative_to(PROJECT).as_posix(),
                           "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
                          for source in runtime],
        "testSelection": "PickSurfaceWorkflowTests;PickSurfaceGeometryTests;PickPlaceToolContactTests;MtcExecutionWorkflowTests;RepeatExecutionWorkflowTests;WorkflowMenuTests",
        "visualExport": "PickSurfaceWorkflowTests.RenderPickSurfacePreviews",
        "transportBoundary": "No transport Start/Connect or network thread; direct contacts, CDR and capability-message injection.",
        "validationPerformed": False,
    }
    report_path.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(f"Prepared pick-surface validation at {destination}")


if __name__ == "__main__":
    main()
