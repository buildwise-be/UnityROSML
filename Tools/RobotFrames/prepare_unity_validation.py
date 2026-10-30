#!/usr/bin/env python3
"""Prepare an isolated Unity project with unchanged prefabs and asset dependencies.

Copies files; never opens the active project or alters source assets. Missing
package GUIDs are left to the real cached URDF package. Actual task/controller,
input, target and visualizer code runs with a ROS connection factory that throws;
the independent QR audit owns hardware detection/placement tests.
"""

import argparse
import json
from pathlib import Path
import re
import shutil


PROJECT = Path(__file__).resolve().parents[2]
DESTINATION = PROJECT / "Library/RosRigValidation"
GUID = re.compile(r"guid:\s*([0-9a-f]{32})")


def copy_file(source):
    relative = source.relative_to(PROJECT)
    target = DESTINATION / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    meta = Path(str(source) + ".meta")
    if meta.is_file():
        shutil.copy2(meta, Path(str(target) + ".meta"))


def main():
    global DESTINATION
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DESTINATION,
                        help="Dedicated isolated project; Library avoids the active Editor cleaning Temp.")
    DESTINATION = parser.parse_args().destination.resolve()
    if DESTINATION == PROJECT or DESTINATION == PROJECT / "Assets" or (DESTINATION / "Assets/Scenes/UR16eInteraction.unity").exists():
        raise ValueError("Refusing to prepare over the active project")
    assets = PROJECT / "Assets"
    index = {}
    for meta in assets.rglob("*.meta"):
        match = GUID.search(meta.read_text(encoding="utf-8", errors="ignore"))
        if match:
            index[match.group(1)] = Path(str(meta)[:-5])
    pending = [assets / "Prefabs/ur16e_kenos/ur16e_kenos.prefab",
               assets / "Prefabs/ur16e_kenos/ur16e_kenos_transparent.prefab"]
    copied, missing = set(), set()
    while pending:
        source = pending.pop()
        if source in copied or not source.is_file():
            continue
        if source.suffix == ".cs":
            # Prefab scripts are provided by the real cached URDF package; any
            # user script is copied only if explicitly listed below.
            continue
        copy_file(source)
        copied.add(source)
        for readable in (source, Path(str(source) + ".meta")):
            if not readable.is_file():
                continue
            for guid in GUID.findall(readable.read_text(encoding="utf-8", errors="ignore")):
                if guid in index:
                    pending.append(index[guid])
                elif not guid.startswith("0000000000000000"):
                    missing.add(guid)

    for folder in ("Assets/Scripts/V3/Frames", "Assets/Scripts/V3/Rendering", "Assets/Tests/RobotFrames", "Assets/Resources/RobotModels", "Assets/Resources/RobotTargetMarkers", "Assets/Resources/RobotContours", "Assets/RosMessages",
                   "Assets/Branding/UnityROSML"):
        for source in (PROJECT / folder).rglob("*"):
            if source.is_file() and source.suffix != ".meta":
                if source.name in ("QrCalibrationTests.cs", "XrAlignmentIntegrationTests.cs"):
                    # The independent hardware-API closure audit runs these actual
                    # QR/placement tests; this project deliberately has inert stubs.
                    stale_test = DESTINATION / source.relative_to(PROJECT)
                    for stale in (stale_test, Path(str(stale_test) + ".meta")):
                        if stale.is_file():
                            if not stale.resolve().is_relative_to(DESTINATION):
                                raise ValueError("Refusing to remove a test outside the isolated project")
                            stale.unlink()
                    continue
                copy_file(source)
    resource_source = PROJECT / "Assets/Resources/RobotModels"
    resource_target = DESTINATION / "Assets/Resources/RobotModels"
    for stale in resource_target.rglob("*"):
        if stale.is_file() and not (resource_source / stale.relative_to(resource_target)).exists():
            if not stale.resolve().is_relative_to(DESTINATION.resolve()):
                raise ValueError("Refusing to remove a file outside the isolated project")
            stale.unlink()
    for runtime in ("Assets/Scripts/V3/ROS/RosRobotRig.cs", "Assets/Scripts/V3/ROS/RobotExecutionState.cs", "Assets/Scripts/V3/ROS/RobotDescriptionMessages.cs",
                    "Assets/Scripts/V3/ROS/ToolSelectionController.cs", "Assets/Scripts/V3/UI/ToolSelectionMenu.cs",
                    "Assets/Scripts/V3/UI/RobotRegistrationGuide.cs",
                    "Assets/Scripts/V3/ROS/TcpPlanningController.cs", "Assets/Scripts/V3/TcpTargetPreview.cs",
                    "Assets/Scripts/V3/URInputManager.cs", "Assets/Scripts/V3/TargetRotationHandles.cs",
                    "Assets/Scripts/V3/TargetPoseContextMenu.cs", "Assets/Scripts/V3/ToolPoseVisual.cs",
                    "Assets/Scripts/V3/NormalTargetMarker.cs",
                    "Assets/Scripts/V3/TargetNameLabel.cs",
                    "Assets/Scripts/V3/TargetMarkerMaterials.cs",
                    "Assets/Scripts/V3/XRTrackingSpaceResolver.cs", "Assets/Scripts/V3/UI/MTCPlanUIManager.cs",
                    "Assets/Scripts/V3/ROS/RobotCoordinateService.cs", "Assets/Scripts/V3/ROS/RobotStateService.cs",
                    "Assets/Scripts/V3/ROS/MTCPlansManager.cs", "Assets/Scripts/V3/ROS/PickPlaceController.cs",
                    "Assets/Scripts/V3/ROS/UR16eTrajectoryVisualizer.cs",
                    "Assets/Samples/Magic Leap SDK/2.6.0/ML Rig & OpenXR Input/MagicLeapOpenXRInput.cs"):
        if (PROJECT / runtime).is_file():
            copy_file(PROJECT / runtime)
    plugin_folder = DESTINATION / "Assets/Plugins"
    plugin_folder.mkdir(parents=True, exist_ok=True)
    for dll in ("Unity.Robotics.ROSTCPConnector.MessageGeneration.dll", "Unity.Robotics.ROSTCPConnector.dll",
                "Unity.Robotics.ROSTCPConnector.Messages.dll"):
        shutil.copy2(PROJECT / "Library/ScriptAssemblies" / dll, plugin_folder / dll)

    cache = PROJECT / "Library/PackageCache"
    wanted = ("com.unity.ext.nunit", "com.unity.test-framework", "com.unity.robotics.urdf-importer",
              "com.unity.editorcoroutines", "com.unity.inputsystem", "com.unity.ugui",
              "com.unity.xr.interaction.toolkit", "com.unity.xr.core-utils", "com.unity.mathematics",
              "com.unity.render-pipelines.universal")
    dependencies = {}
    pending_packages = list(wanted)
    while pending_packages:
        name = pending_packages.pop()
        if name in dependencies:
            continue
        if name.startswith("com.unity.modules."):
            dependencies[name] = "1.0.0"
            continue
        matches = sorted(cache.glob(name + "@*"))
        if len(matches) != 1:
            raise ValueError(f"Expected one cached {name}, found {len(matches)}")
        dependencies[name] = "file:" + matches[0].as_posix()
        manifest = json.loads((matches[0] / "package.json").read_text(encoding="utf-8-sig"))
        pending_packages.extend(manifest.get("dependencies", {}))
    for module in ("physics", "jsonserialize", "imgui", "uielements", "ui", "animation", "imageconversion",
                   "audio", "unitywebrequest", "xr"):
        dependencies["com.unity.modules." + module] = "1.0.0"
    package_folder = DESTINATION / "Packages"
    package_folder.mkdir(parents=True, exist_ok=True)
    (package_folder / "manifest.json").write_text(json.dumps({"dependencies": dependencies}, indent=2) + "\n")
    font_assets = PROJECT / "Assets/TextMesh Pro"
    if font_assets.is_dir():
        shutil.copytree(font_assets, DESTINATION / "Assets/TextMesh Pro", dirs_exist_ok=True)
    # These workflow stubs exist only in the isolated harness. They keep the
    # actual controller policy/UI testable without compiling unrelated robot
    # command workflows or permitting an accidental ROS connection.
    stub_file = DESTINATION / "Assets/OfflineValidationWorkflowStubs.cs"
    stub_file.write_text('''using System;
using UnityEngine;
using UnityEngine.Events;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
public sealed class ROSService {
    public static ROSService Instance => throw new InvalidOperationException("Offline validation must never request a ROS connection.");
    public ROSConnection GetROSConnection() => throw new InvalidOperationException("Offline validation has no ROS connection.");
    public void Subscribe<T>(string topic, Action<T> callback) where T : Message, new() => throw new InvalidOperationException("Offline validation has no subscriptions.");
    public void RegisterPublisher<T>(string topic) where T : Message, new() => throw new InvalidOperationException("Offline validation has no publishers.");
    public void Publish(string topic, Message message) => throw new InvalidOperationException("Offline validation must never publish a command.");
}
public sealed class RemoteExecutionController : MonoBehaviour {
    public void SetPoseSources(Transform pick, Transform place) {}
}
// Hardware marker detection and real placement are validated in the separate QR
// audit. Their public UI surface is inert in this connection-free test project.
public sealed class QRDetector : MonoBehaviour {
    public bool HasValidatedPose => false;
    public Transform TrackingSpace => null;
    public string StatusText => "Offline QR validation";
    public float MarkerSizeMeters => 0.087f;
    public string ExpectedMarkerPayload => "";
    public string DetectedPayload => "";
    public event Action StateChanged;
    public void Rescan() { StateChanged?.Invoke(); }
}
public sealed class URInstantiator : MonoBehaviour {
    public UnityEvent<GameObject, Pose> OnURInstantiated = new UnityEvent<GameObject, Pose>();
    public GameObject RobotPrefab { get; set; }
    public string CalibrationSummary => "Offline placement validation";
    public bool IsCalibrating { get; private set; }
    public bool HasSavedCalibration => false;
    public bool CanCalibrate => false;
    public string AlignmentStatus => "Place the robot before aligning it.";
    public event Action CalibrationStateChanged;
    public void BeginCalibration() { CalibrationStateChanged?.Invoke(); }
    public void NudgePosition(Vector3 value) { CalibrationStateChanged?.Invoke(); }
    public void RotateBase(Vector3 axis, float angle) { CalibrationStateChanged?.Invoke(); }
    public void SaveCalibration() { CalibrationStateChanged?.Invoke(); }
    public void CancelCalibration() { IsCalibrating = false; CalibrationStateChanged?.Invoke(); }
    public void RestoreDefaultCalibration() { CalibrationStateChanged?.Invoke(); }
    public void InstantiateUR() => throw new InvalidOperationException("Offline validation must never place a robot.");
}
// Exercises the actual public Execute path without constructing ROS transport.
public sealed class OfflineMtcExecutionController : PickPlaceController {
    public int PublishCount;
    public int RepeatPublishCount;
    public string LastRepeatPayload;
    public bool PublishSucceeds = true;
    protected override bool TryPublishExecutionRequest(out string reason) {
        PublishCount++;
        reason = PublishSucceeds ? null : "Offline transport refused request";
        return PublishSucceeds;
    }
    protected override bool TryPublishRepeatExecutionRequest(string payload, out string reason) {
        PublishCount++;
        RepeatPublishCount++;
        LastRepeatPayload = payload;
        reason = PublishSucceeds ? null : "Offline repeat transport refused request";
        return PublishSucceeds;
    }
}
public sealed class OfflineTcpExecutionController : TcpPlanningController {
    public int PublishCount;
    public string LastTopic;
    public string LastPayload;
    public bool PublishSucceeds = true;
    protected override bool IsConnected => true;
    protected override bool TryPublishExecution(string topic, string payload, out string reason) {
        PublishCount++;
        LastTopic = topic;
        LastPayload = payload;
        reason = PublishSucceeds ? null : "Offline TCP transport refused request";
        return PublishSucceeds;
    }
}
''')
    settings = DESTINATION / "ProjectSettings"
    settings.mkdir(parents=True, exist_ok=True)
    shutil.copy2(PROJECT / "ProjectSettings/ProjectVersion.txt", settings / "ProjectVersion.txt")
    for asset in ("Mobile_RPAsset.asset", "Mobile_Renderer.asset", "PC_RPAsset.asset", "PC_Renderer.asset", "UniversalRenderPipelineGlobalSettings.asset"):
        copy_file(PROJECT / "Assets/Settings" / asset)
    graphics = (PROJECT / "ProjectSettings/GraphicsSettings.asset").read_text(encoding="utf-8-sig")
    mobile_guid = GUID.search((PROJECT / "Assets/Settings/Mobile_RPAsset.asset.meta").read_text()).group(1)
    graphics = re.sub(r"  m_CustomRenderPipeline:.*", f"  m_CustomRenderPipeline: {{fileID: 11400000, guid: {mobile_guid}, type: 2}}", graphics)
    (settings / "GraphicsSettings.asset").write_text(graphics, encoding="utf-8")
    (settings / "ProjectSettings.asset").write_text("%YAML 1.1\n%TAG !u! tag:unity3d.com,2011:\n--- !u!129 &1\nPlayerSettings:\n  m_ObjectHideFlags: 0\n  serializedVersion: 28\n  activeInputHandler: 1\n  scriptingDefineSymbols:\n    Standalone: ROS2\n", encoding="utf-8")
    (DESTINATION / "preparation-report.json").write_text(json.dumps({
        "sourceProject": str(PROJECT), "copiedPrefabAssetCount": len(copied),
        "unresolvedGuidsProvidedByPackages": sorted(missing)}, indent=2) + "\n")
    print(f"Prepared {DESTINATION}; {len(copied)} unchanged prefab assets and dependencies copied")
    print(f"Package GUIDs not in Assets: {len(missing)}")


if __name__ == "__main__":
    main()
