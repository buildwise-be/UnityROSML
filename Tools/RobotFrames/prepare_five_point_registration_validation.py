#!/usr/bin/env python3
"""Prepare the five-point placement and bounded correction workflow offline.

Copies real registration/controller/menu/guide code and exact app-scene
snapshots. The ROS factory throws, and QR hardware remains an inert stub.
Existing cached projects can be reused with --destination.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import prepare_scene_workflow_validation as scenes
import prepare_unity_validation as common


PROJECT = common.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/FivePointRegistrationValidation"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    destination = parser.parse_args().destination.resolve()
    library = (PROJECT / "Library").resolve()
    if destination == library or not destination.is_relative_to(library):
        raise ValueError("Five-point validation requires a dedicated project below Library")
    arguments = sys.argv[:]
    try:
        sys.argv = [sys.argv[0], "--destination", str(destination), "--stub-transport"]
        scenes.main()
    finally:
        sys.argv = arguments

    menu_sources = sorted((PROJECT / "Assets/Scripts/V3/UI").glob("ToolSelectionMenu*.cs"))
    for source in menu_sources:
        common.copy_file(source)
    report_path = destination / "preparation-report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["fivePointRegistrationHarness"] = {
        "actualMenuSources": [{"path": source.relative_to(PROJECT).as_posix(),
                               "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
                              for source in menu_sources],
        "landmarkContract": "Base cover (lowest blue cover), Shoulder, Elbow, Wrist 2, Wrist 3, from base to tool",
        "correctionContract": "Robot-local XYZ position correction; total displacement from original fit limited to 20 mm",
        "miniatureContract": "One complete revolution per 10 seconds, preserving highlighted landmark and locked joint pose",
        "transportBoundary": "The ROS factory throws before any connection, subscription or publisher; no native XR scene is loaded",
        "validationPerformed": False,
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared five-point placement validation at {destination}")


if __name__ == "__main__":
    main()
