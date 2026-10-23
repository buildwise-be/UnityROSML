#!/usr/bin/env python3
"""Prepare actual QR/alignment components with a transport factory that throws.

Uses cached local packages and assemblies. Does not open the active Unity project,
start an XR session, connect to ROS, or write calibration into the real app store.
"""

import argparse
import json
from pathlib import Path
import shutil
import sys

import prepare_unity_validation as common


PROJECT = common.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/XrAlignmentValidation"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    destination = parser.parse_args().destination.resolve()
    sys.argv = [sys.argv[0], "--destination", str(destination)]
    common.main()

    runtime = ["Assets/Scripts/V3/URInstantiator.cs",
               "Assets/Scripts/V3/ROS/RobotCoordinateService.cs",
               "Assets/Scripts/V3/ROS/RobotStateService.cs",
               "Assets/Scripts/V3/ROS/RosRobotRig.cs",
               "Assets/Scripts/V3/XRTrackingSpaceResolver.cs"]
    runtime.extend(path.relative_to(PROJECT).as_posix()
                   for path in (PROJECT / "Assets/Scripts/V3/QR").glob("*.cs"))
    for relative in runtime:
        common.copy_file(PROJECT / relative)

    # The task workflow is checked by the other isolated project. These stubs
    # cannot connect to ROS or execute any command, and this suite never runs Start.
    (destination / "Assets/OfflineValidationWorkflowStubs.cs").write_text('''using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
public sealed class ROSService {
    public static ROSService Instance => throw new InvalidOperationException("Alignment validation must never request a ROS connection.");
    public ROSConnection GetROSConnection() => throw new InvalidOperationException("Alignment validation has no ROS transport.");
}
public sealed class UR16eTrajectoryVisualizer : MonoBehaviour { public void SetTrajectory(object value) {} }
public sealed class PickPlaceController : MonoBehaviour { public void InvalidatePlanningRequest() {} }
public sealed class MTCPlansManager : MonoBehaviour { public void InvalidatePlan() {} }
public sealed class TcpPlanningController : MonoBehaviour { public void InvalidatePlan(string reason = "") {} }
public sealed class TargetRotationHandles : MonoBehaviour { public static void CancelAllInteractions() {} }
''', encoding="utf-8")
    # Remove only known workflow files previously copied into this dedicated harness.
    for relative in ("Assets/Scripts/V3/ROS/TcpPlanningController.cs", "Assets/Scripts/V3/ROS/MTCPlansManager.cs",
                     "Assets/Scripts/V3/ROS/UR16eTrajectoryVisualizer.cs", "Assets/Scripts/V3/URInputManager.cs",
                     "Assets/Scripts/V3/TcpTargetPreview.cs", "Assets/Scripts/V3/TargetRotationHandles.cs",
                     "Assets/Scripts/V3/UI/MTCPlanUIManager.cs"):
        source = destination / relative
        for owned in (source, Path(str(source) + ".meta")):
            if owned.is_file():
                if not owned.resolve().is_relative_to(destination):
                    raise ValueError("Refusing to remove a file outside this isolated project")
                owned.unlink()
    (destination / "Assets/Scripts/V3/UI/ToolSelectionMenu.cs").write_text(
        "using UnityEngine; public sealed class ToolSelectionMenu : MonoBehaviour { public void PositionAt(Pose pose) {} }\n",
        encoding="utf-8")
    (destination / "Assets/Scripts/V3/ROS/ToolSelectionController.cs").write_text(
        "using UnityEngine; public sealed class ToolSelectionController : MonoBehaviour { public static ToolSelectionController Instance => null; "
        "public bool ConfigureRobot(RosRobotRig rig) => rig.ApplyModel(RosRobotModelCatalog.Load().Find(\"kenos\")); }\n",
        encoding="utf-8")

    tests = {"QrCalibrationTests.cs", "XrAlignmentIntegrationTests.cs", "CalibrationProfileTests.cs"}
    for test in (destination / "Assets/Tests/RobotFrames").glob("*.cs"):
        if test.name not in tests:
            for owned in (test, Path(str(test) + ".meta")):
                if owned.is_file():
                    if not owned.resolve().is_relative_to(destination):
                        raise ValueError("Refusing to remove a test outside this isolated project")
                    owned.unlink()
    for name in tests:
        source = PROJECT / "Assets/Tests/RobotFrames" / name
        if source.is_file():
            common.copy_file(source)

    plugins = destination / "Assets/Plugins"
    for name in ("MagicLeap.SDK", "Unity.XR.OpenXR", "Unity.XR.ARFoundation", "Unity.XR.ARFoundation.InternalUtils",
                 "Unity.XR.ARSubsystems", "Unity.XR.InteractionSubsystems", "Unity.XR.Simulation"):
        shutil.copy2(PROJECT / "Library/ScriptAssemblies" / (name + ".dll"), plugins / (name + ".dll"))
    manifest_file = destination / "Packages/manifest.json"
    manifest = json.loads(manifest_file.read_text())
    for name in ("com.unity.collections", "com.unity.burst", "com.unity.render-pipelines.core",
                 "com.unity.render-pipelines.universal", "com.unity.xr.management"):
        matches = list((PROJECT / "Library/PackageCache").glob(name + "@*"))
        if len(matches) != 1:
            raise ValueError(f"Expected one cached {name}")
        manifest["dependencies"][name] = "file:" + matches[0].as_posix()
    manifest_file.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared actual-component XR alignment validation at {destination}")


if __name__ == "__main__":
    main()
