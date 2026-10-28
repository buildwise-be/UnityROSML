#!/usr/bin/env python3
"""Prepare actual point-registration components in an isolated Unity project.

Reuses the ordinary workflow harness and its real UI, rig, planning gates and
input policy. Only QR hardware is inert, and ROSService still throws before any
transport can be obtained. Does not open or alter the active Unity project.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

import prepare_unity_validation as common


PROJECT = common.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/PointRegistrationValidation"


def replace_stub(text, name, replacement):
    pattern = rf"^public sealed class {re.escape(name)} : MonoBehaviour \{{\n.*?^\}}\n"
    changed, count = re.subn(pattern, lambda _: replacement, text, flags=re.MULTILINE | re.DOTALL)
    if count != 1:
        raise ValueError(f"Expected exactly one generated {name} stub, found {count}")
    return changed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    destination = parser.parse_args().destination.resolve()
    library = (PROJECT / "Library").resolve()
    if destination == library or not destination.is_relative_to(library):
        raise ValueError("Point-registration validation must use a dedicated subfolder of this project's Library")

    original_arguments = sys.argv[:]
    try:
        sys.argv = [sys.argv[0], "--destination", str(destination)]
        common.main()
    finally:
        sys.argv = original_arguments

    # Replace only known generated classes. The actual placement/subscriber code
    # and new registration workflow are compiled unchanged beside the full UI.
    stubs_path = destination / "Assets/OfflineValidationWorkflowStubs.cs"
    stubs = stubs_path.read_text(encoding="utf-8")
    stubs = replace_stub(stubs, "URInstantiator", "")
    stubs = replace_stub(stubs, "QRDetector", '''public sealed class QRDetector : MonoBehaviour {
    public UnityEvent<Pose> OnQRValidated = new UnityEvent<Pose>();
    public bool HasValidatedPose { get; set; }
    public Transform TrackingSpace { get; set; }
    public Pose StabilizedPose { get; set; } = Pose.identity;
    public int ValidationRevision { get; set; }
    public string StatusText => "Offline QR hardware stub";
    public float MarkerSizeMeters => 0.087f;
    public string ExpectedMarkerPayload => "";
    public string DetectedPayload { get; set; } = "";
    public event Action StateChanged;
    public void Rescan() { HasValidatedPose = false; StateChanged?.Invoke(); }
    public void ResetDetection() { Rescan(); }
}
''')
    stubs_path.write_text(stubs, encoding="utf-8")

    runtime = ["Assets/Scripts/V3/URInstantiator.cs",
               "Assets/Scripts/V3/RobotPointRegistration.cs",
               "Assets/Scripts/V3/ROS/JointStateSubscriber.cs"]
    for relative in runtime:
        source = PROJECT / relative
        if not source.is_file():
            raise FileNotFoundError(f"Required point-registration source is missing: {relative}")
        common.copy_file(source)

    report_path = destination / "preparation-report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["pointRegistrationHarness"] = {
        "actualAdditionalRuntimeSources": [
            {"path": relative, "sha256": hashlib.sha256((PROJECT / relative).read_bytes()).hexdigest()}
            for relative in runtime
        ],
        "hardwareStub": "QRDetector starts unvalidated and allows synthetic marker injection; it has no XR API or marker detector.",
        "transportBoundary": "ROSService throws before obtaining any ROS connection, subscription or publisher.",
        "sourceCopyPolicy": "Frames helpers, tests, model assets and the real workflow/UI are copied by the common preparer.",
        "validationPerformed": False,
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared actual point-registration components at {destination}")


if __name__ == "__main__":
    main()
