#!/usr/bin/env python3
"""Prepare a separate offline Android contour player-data validation project.

Reuses the real point-registration workflow harness and its cached packages.
Copies the export helper into this isolated project's Assets/Editor; it never
opens Unity, runs Gradle, installs a player, or starts XR/robot transport.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys

import prepare_point_registration_validation as point


PROJECT = point.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/RobotContourPlayerValidation"
HELPER_NAME = "RobotContourPlayerValidation.cs"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    destination = parser.parse_args().destination.resolve()
    library = (PROJECT / "Library").resolve()
    if destination.parent != library:
        raise ValueError("Contour player validation must use a dedicated project directly under this project's Library")
    if destination.name in ("PointRegistrationValidation", "RosRigValidation"):
        raise ValueError("Use a separate contour player project so graphics/workflow validation can run concurrently")

    original_arguments = sys.argv[:]
    try:
        sys.argv = [sys.argv[0], "--destination", str(destination)]
        point.main()
    finally:
        sys.argv = original_arguments

    # The common EditMode harness selects ROS2 for Standalone. Android must use
    # the same message contract before its first script compilation as well.
    settings = destination / "ProjectSettings/ProjectSettings.asset"
    settings.write_text(settings.read_text(encoding="utf-8") + "    Android: ROS2\n", encoding="utf-8")

    # EditMode uses cached editor DLLs for convenience. A player must compile
    # the actual cached connector package so UNITY_EDITOR code is excluded.
    connector_matches = sorted((PROJECT / "Library/PackageCache").glob("com.unity.robotics.ros-tcp-connector@*"))
    if len(connector_matches) != 1:
        raise ValueError("Expected one cached ROS TCP Connector package")
    manifest_path = destination / "Packages/manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["dependencies"]["com.unity.robotics.ros-tcp-connector"] = "file:" + connector_matches[0].as_posix()
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    for name in ("Unity.Robotics.ROSTCPConnector", "Unity.Robotics.ROSTCPConnector.MessageGeneration",
                 "Unity.Robotics.ROSTCPConnector.Messages"):
        for suffix in (".dll", ".dll.meta"):
            (destination / "Assets/Plugins" / (name + suffix)).unlink(missing_ok=True)

    helper = Path(__file__).with_name(HELPER_NAME)
    target = destination / "Assets/Editor" / HELPER_NAME
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(helper, target)
    sources = [
        path for folder in ("Assets/Scripts/V3/Rendering", "Assets/Resources/RobotContours")
        for path in sorted((PROJECT / folder).rglob("*")) if path.is_file()
    ]
    sources.extend(PROJECT / relative for relative in (
        "Assets/Scripts/V3/ROS/RosRobotRig.cs", "Assets/Scripts/V3/ROS/RobotStateService.cs",
        "Assets/Scripts/V3/ROS/RobotExecutionState.cs", "Assets/Settings/Mobile_RPAsset.asset",
        "Assets/Settings/Mobile_Renderer.asset", "Assets/Settings/UniversalRenderPipelineGlobalSettings.asset"))
    source_hashes = []
    for source in sources:
        relative = source.relative_to(PROJECT)
        copied = destination / relative
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        if not copied.is_file() or hashlib.sha256(copied.read_bytes()).hexdigest() != digest:
            raise ValueError("Contour export source copy differs: " + relative.as_posix())
        source_hashes.append({"path": relative.as_posix(), "sha256": digest})

    report_path = destination / "preparation-report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["contourPlayerHarness"] = {
        "sourceFiles": source_hashes,
        "editorHelper": HELPER_NAME,
        "editorHelperSha256": hashlib.sha256(helper.read_bytes()).hexdigest(),
        "executeMethod": "RobotContourPlayerValidation.Export",
        "result": "contour-player-validation-result.json",
        "exportDirectory": "contour-export",
        "fullPlayerDataRequested": True,
        "graphicsApi": "Vulkan",
        "scriptingBackend": "Mono2x",
        "architecture": "ARMv7",
        "gradleBuildRequested": False,
        "deviceDeploymentRequested": False,
        "xrRuntimeRequested": False,
        "robotTransportRequested": False,
        "connectorSource": connector_matches[0].as_posix(),
        "validationPerformed": False,
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared independent contour player-data validation: {destination}")
    print("Launch Unity with -buildTarget Android -executeMethod RobotContourPlayerValidation.Export")


if __name__ == "__main__":
    main()
