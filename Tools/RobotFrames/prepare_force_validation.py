#!/usr/bin/env python3
"""Prepare real wrench subscriber and task-menu checks without robot hardware.

The inherited scene harness copies the exact runtime, menus, tests and cached
connector assemblies. Tests construct inactive components and inject messages;
they never start a ROS connection, alter a controller or request robot motion.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import prepare_scene_workflow_validation as scene
import prepare_unity_validation as common


PROJECT = common.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/ForceTorqueValidation"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    destination = parser.parse_args().destination.resolve()
    library = (PROJECT / "Library").resolve()
    if destination == library or not destination.is_relative_to(library):
        raise ValueError("Force validation requires a dedicated project below Library")
    original_arguments = sys.argv[:]
    try:
        sys.argv = [sys.argv[0], "--destination", str(destination)]
        scene.main()
    finally:
        sys.argv = original_arguments
    runtime = sorted({PROJECT / "Assets/Scripts/V3/ROS/ForceTorqueSubscriber.cs"} |
                     set((PROJECT / "Assets/Scripts/V3/UI").glob("ToolSelectionMenu*.cs")))
    for source in runtime:
        common.copy_file(source)
    report_path = destination / "preparation-report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["forceTorqueHarness"] = {
        "actualRuntimeSources": [
            {"path": source.relative_to(PROJECT).as_posix(),
             "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
            for source in runtime
        ],
        "testSelection": ("ForceTorqueSubscriberTests;WorkflowMenuTests;WorkflowServiceLifecycleTests;"
                          "TcpPlanningControllerTests;FixtureTilingWorkflowTests;MtcExecutionWorkflowTests;"
                          "RepeatExecutionWorkflowTests;PickPlaceToolContactTests"),
        "transportBoundary": "Inactive connector/subscriber components and direct synthetic message injection; no Connect or transport Start call.",
        "validationPerformed": False,
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared force/torque validation at {destination}")


if __name__ == "__main__":
    main()
