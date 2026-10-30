#!/usr/bin/env python3
"""Prepare the real fixture-tiling workflow in a connection-free Unity project."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import prepare_point_registration_validation as registration
import prepare_unity_validation as common


PROJECT = common.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/FixtureTilingValidation"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    destination = parser.parse_args().destination.resolve()
    library = (PROJECT / "Library").resolve()
    if destination == library or not destination.is_relative_to(library):
        raise ValueError("Fixture validation requires a dedicated project below Library")

    original_arguments = sys.argv[:]
    try:
        sys.argv = [sys.argv[0], "--destination", str(destination)]
        registration.main()
    finally:
        sys.argv = original_arguments

    runtime = sorted(set(
        source for folder in ("Assets/Scripts/V3/ROS", "Assets/Scripts/V3/UI")
        for source in (PROJECT / folder).glob("Fixture*.cs")
    ) | set((PROJECT / "Assets/Scripts/V3/UI").glob("*.Fixture.cs")))
    for source in runtime:
        common.copy_file(source)
    stubs_path = destination / "Assets/OfflineValidationWorkflowStubs.cs"
    with stubs_path.open("a", encoding="utf-8") as stubs:
        stubs.write('''
public sealed class OfflineFixtureTilingController : FixtureTilingController {
    public int PublishCount;
    public int PlanPublishCount;
    public string LastPlanId;
    protected override bool Connected => true;
    protected override bool TryPublishPlan(RosMessageTypes.Ur16eInterfaces.FixtureTilingTaskMsg request, out string reason) {
        PlanPublishCount++; reason = null; return true;
    }
    protected override bool TryPublishExecution(string planId, out string reason) {
        PublishCount++; LastPlanId = planId; reason = null; return true;
    }
}
''')
    for stale in (destination / "Assets/RosMessages").rglob("*"):
        if stale.is_file() and not (PROJECT / stale.relative_to(destination)).exists():
            if not stale.resolve().is_relative_to(destination):
                raise ValueError("Refusing to remove stale schema outside the isolated project")
            stale.unlink()

    report_path = destination / "preparation-report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["fixtureTilingHarness"] = {
        "actualAdditionalRuntimeSources": [
            {"path": source.relative_to(PROJECT).as_posix(),
             "sha256": hashlib.sha256(source.read_bytes()).hexdigest()}
            for source in runtime
        ],
        "transportBoundary": "ROSService throws before any connection; tests inject callbacks and override publication.",
        "sourceCopyPolicy": "Actual fixture runtime, all RobotFrames tests and ROS message definitions are copied unchanged.",
        "validationPerformed": False,
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared fixture-tiling validation at {destination}")


if __name__ == "__main__":
    main()
