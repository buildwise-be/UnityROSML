#!/usr/bin/env python3
"""Prepare an offline, isolated Android player-data test for runtime marker materials.

Uses the active project's exact cached URP/package versions and unchanged Resources
materials. The generated Unity entry point exports Mono/ARMv7 Gradle player data;
it does not run Gradle, install an APK, start XR, or open the active project.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil


PROJECT = Path(__file__).resolve().parents[2]


def prepare(destination: Path) -> None:
    destination = destination.resolve()
    allowed = (PROJECT / "Library/MarkerPlayerValidation").resolve()
    if destination != allowed:
        raise ValueError(f"Dedicated validation destination must be {allowed}")
    if (destination / "Assets/Scenes/UR16eInteraction.unity").exists():
        raise ValueError("Refusing to prepare over the active project")

    copied = []

    def copy(relative):
        source = PROJECT / relative
        if not source.is_file():
            raise FileNotFoundError(source)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        copied.append({"path": relative, "sha256": hashlib.sha256(source.read_bytes()).hexdigest()})

    settings = destination / "ProjectSettings"
    settings.mkdir(parents=True, exist_ok=True)
    content = (PROJECT / "ProjectSettings/ProjectSettings.asset").read_text(encoding="utf-8-sig")
    content = re.sub(r"(?m)^(  useCustomMainManifest: )1$", r"\g<1>0", content)
    content = re.sub(r"(?m)^(  activeInputHandler: )\d+$", r"\g<1>0", content)
    # This harness has no input package or robot scene. Keep SDK/JDK preferences
    # untouched: Unity uses the same installed standard-user tools as the app.
    (settings / "ProjectSettings.asset").write_text(content, encoding="utf-8")
    for relative in ("ProjectSettings/ProjectVersion.txt", "ProjectSettings/GraphicsSettings.asset",
                     "ProjectSettings/QualitySettings.asset", "ProjectSettings/URPProjectSettings.asset"):
        copy(relative)
    for source in (PROJECT / "Assets/Settings").glob("*.asset*"):
        copy(source.relative_to(PROJECT).as_posix())
    copy("Assets/Resources/RobotTargetMarkers.meta")
    for source in sorted((PROJECT / "Assets/Resources/RobotTargetMarkers").glob("*")):
        if source.is_file():
            copy(source.relative_to(PROJECT).as_posix())
    copy("Assets/Scripts/V3/TargetMarkerMaterials.cs")
    copy("Assets/Scripts/V3/TargetMarkerMaterials.cs.meta")

    cache = PROJECT / "Library/PackageCache"
    packages = {}
    for source in sorted(cache.glob("*/package.json")):
        package = json.loads(source.read_text(encoding="utf-8-sig"))
        packages[package["name"]] = (source.parent, package)
    dependencies = {}
    pending = ["com.unity.render-pipelines.universal", "com.unity.modules.androidjni",
               "com.unity.modules.imageconversion", "com.unity.modules.jsonserialize",
               "com.unity.modules.xr"]
    versions = {}
    while pending:
        name = pending.pop()
        if name in dependencies:
            continue
        if name.startswith("com.unity.modules."):
            dependencies[name] = "1.0.0"
            continue
        path, package = packages[name]
        dependencies[name] = "file:" + path.as_posix()
        versions[name] = package["version"]
        pending.extend(package.get("dependencies", {}).keys())
    if versions["com.unity.render-pipelines.universal"] != "17.0.4":
        raise ValueError("Expected the app's current URP17.0.4")
    manifest = destination / "Packages/manifest.json"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps({"dependencies": dict(sorted(dependencies.items()))}, indent=2) + "\n")
    editor = destination / "Assets/Editor/MarkerPlayerExportValidation.cs"
    editor.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(Path(__file__).with_name("MarkerPlayerExportValidation.cs"), editor)
    (destination / "preparation-report.json").write_text(json.dumps({
        "sourceProject": str(PROJECT), "destination": str(destination),
        "sourceFiles": copied, "cachedPackageVersions": versions,
        "editorHelperSha256": hashlib.sha256(Path(__file__).with_name("MarkerPlayerExportValidation.cs").read_bytes()).hexdigest(),
        "gradleBuildRequested": False, "deviceDeploymentRequested": False,
        "xrRuntimeRequested": False,
        "changesInIsolatedCopyOnly": ["custom manifest disabled", "legacy input",
                                      "no preloaded input assets", "Mono ARMv7 full player-data export"],
    }, indent=2) + "\n")
    print(f"Prepared {destination}: {len(copied)} source files; {len(dependencies)} local/builtin packages")


def record(destination: Path) -> None:
    destination = destination.resolve()
    if destination != (PROJECT / "Library/MarkerPlayerValidation").resolve():
        raise ValueError("Only the dedicated validation project can be recorded")
    preparation = json.loads((destination / "preparation-report.json").read_text(encoding="utf-8-sig"))
    result = json.loads((destination / "marker-validation-result.json").read_text(encoding="utf-8-sig"))
    if not result["success"]:
        raise ValueError(result.get("error", "Player-data verification did not succeed"))
    for item in preparation["sourceFiles"]:
        if hashlib.sha256((PROJECT / item["path"]).read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError("Source changed since validation preparation: " + item["path"])
    helper = Path(__file__).with_name("MarkerPlayerExportValidation.cs")
    if helper.read_bytes() != (destination / "Assets/Editor/MarkerPlayerExportValidation.cs").read_bytes():
        raise ValueError("Editor helper changed since validation preparation")
    preparation["editorHelperSha256"] = hashlib.sha256(helper.read_bytes()).hexdigest()
    log_path = destination / "marker-editor.log"
    log = log_path.read_text(encoding="utf-8-sig", errors="replace")
    compiled = re.findall(r'Compiling shader "Universal Render Pipeline/Unlit" pass "([^"]*)"', log)
    if not compiled or "Serialized binary data for shader Universal Render Pipeline/Unlit" not in log:
        raise ValueError("No compiled/serialized Android Unlit shader evidence in the build log")
    compilation = []
    for pass_name, section in re.findall(
            r'Compiling shader "Universal Render Pipeline/Unlit" pass "([^"]*)"[^\n]*\n(.*?)'
            r'(?=Compiling shader "|Serialized binary data for shader Universal Render Pipeline/Unlit)', log, re.S):
        counts = re.findall(r'compiled (\d+) variants', section)
        compilation.append({"pass": pass_name, "compiledVariants": sum(map(int, counts))})
    programs = re.search(r'Serialized binary data for shader Universal Render Pipeline/Unlit[^\n]*\n\s*vulkan \(total internal programs: (\d+), unique: (\d+)\)', log)
    if not any(item["pass"] == "Unlit" and item["compiledVariants"] > 0 for item in compilation) or not programs:
        raise ValueError("Expected newly compiled Vulkan Unlit forward variants and serialized programs")
    export = destination / "marker-export/unityLibrary"
    gradle = (export / "build.gradle").read_text(encoding="utf-8-sig")
    mono_dll = export / "src/main/assets/bin/Data/Managed/Assembly-CSharp.dll"
    if 'abiFilters "armeabi-v7a"' not in gradle or not mono_dll.is_file():
        raise ValueError("Expected Mono ARMv7 player export artifacts")
    if "org.gradle.launcher.GradleMain" in log or "Application installed to device" in log:
        raise ValueError("Validation must not run Gradle or install a player")
    summary = {
        "recordedUtc": datetime.now(timezone.utc).isoformat(),
        "command": 'Unity.exe -batchmode -nographics -quit -projectPath Library/MarkerPlayerValidation -buildTarget Android -executeMethod MarkerPlayerExportValidation.Export -logFile Library/MarkerPlayerValidation/marker-editor.log',
        "preparation": preparation, "result": result,
        "compiledUnlitPasses": compiled,
        "unlitCompilation": compilation,
        "serializedVulkanPrograms": int(programs.group(1)),
        "uniqueSerializedVulkanPrograms": int(programs.group(2)),
        "editorLogSha256": hashlib.sha256(log_path.read_bytes()).hexdigest(),
        "fullPlayerDataBuild": True, "mainProjectOpenedByValidation": False,
        "exportArtifacts": {"monoAssemblyCSharpDll": True, "gradleAbi": "armeabi-v7a"},
        "proofLimit": result["limitation"],
    }
    output = Path(__file__).with_name("marker_player_validation_summary.json")
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded successful Android player-data verification: {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=PROJECT / "Library/MarkerPlayerValidation")
    parser.add_argument("--record", action="store_true", help="Record completed Unity result and matching source hashes")
    arguments = parser.parse_args()
    (record if arguments.record else prepare)(arguments.destination)
