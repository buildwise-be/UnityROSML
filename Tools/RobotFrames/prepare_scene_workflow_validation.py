#!/usr/bin/env python3
"""Prepare scene routing and lifecycle checks without importing hardware scenes.

Uses real runtime services and navigation. Scene YAML/build settings are copied
as text resources with source hashes, so tests inspect exact serialized assets
without loading XR prefabs or invoking their native lifecycle methods.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

import prepare_fixture_tiling_validation as fixture
import prepare_unity_validation as common


PROJECT = common.PROJECT
DEFAULT_DESTINATION = PROJECT / "Library/SceneWorkflowValidation"
SCENES = ("Welcome", "SimplePickAndPlace", "RepetitivePickAndPlace")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    parser.add_argument("--stub-transport", action="store_true",
                        help="Use the throwing ROS factory for the complete baseline workflow suite")
    options = parser.parse_args()
    destination = options.destination.resolve()
    library = (PROJECT / "Library").resolve()
    if destination == library or not destination.is_relative_to(library):
        raise ValueError("Scene validation requires a dedicated project below Library")
    arguments = sys.argv[:]
    try:
        sys.argv = [sys.argv[0], "--destination", str(destination)]
        fixture.main()
    finally:
        sys.argv = arguments

    if not options.stub_transport:
        stubs_path = destination / "Assets/OfflineValidationWorkflowStubs.cs"
        stubs = stubs_path.read_text(encoding="utf-8")
        stubs, replacements = re.subn(r"^public sealed class ROSService \{\n.*?^\}\n", "", stubs,
                                     flags=re.MULTILINE | re.DOTALL)
        if replacements != 1:
            raise ValueError("Expected exactly one ordinary harness ROSService stub")
        stubs_path.write_text(stubs, encoding="utf-8")

    scripts = PROJECT / "Assets/Scripts"
    runtime = set(scripts.rglob("RobotApp*.cs")) | set(scripts.rglob("*Welcome*.cs")) | {
        PROJECT / "Assets/Scripts/V3/ROS/ROSService.cs",
        PROJECT / "Assets/Scripts/V3/RemoteExecution/RemoteExecutorClient.cs",
        PROJECT / "Assets/Scripts/V3/RemoteExecution/RemotePoseMessages.cs",
    }
    runtime.update((scripts / "V3/UI").glob("*.Navigation.cs"))
    if options.stub_transport:
        runtime.remove(PROJECT / "Assets/Scripts/V3/ROS/ROSService.cs")
        stale = destination / "Assets/Scripts/V3/ROS/ROSService.cs"
        for path in (stale, Path(str(stale) + ".meta")):
            if path.is_file():
                if not path.resolve().is_relative_to(destination):
                    raise ValueError("Refusing to remove a source outside the validation project")
                path.unlink()
    for source in sorted(runtime):
        common.copy_file(source)

    resources = destination / "Assets/Tests/RobotFrames/Resources/RobotAppScenes"
    resources.mkdir(parents=True, exist_ok=True)
    sources = []
    for name in SCENES:
        scene = PROJECT / "Assets/Scenes" / (name + ".unity")
        meta = Path(str(scene) + ".meta")
        if not scene.is_file() or not meta.is_file():
            raise FileNotFoundError("Required application scene is missing: " + str(scene))
        text = resources / (name + ".txt")
        text.write_bytes(scene.read_bytes())
        guid = re.search(r"^guid:\s*([0-9a-f]{32})", meta.read_text(), re.MULTILINE).group(1)
        sources.append({"name": name, "path": scene.relative_to(PROJECT).as_posix(),
                        "resource": "RobotAppScenes/" + name, "sha256": digest(scene), "guid": guid})
    build = PROJECT / "ProjectSettings/EditorBuildSettings.asset"
    (resources / "EditorBuildSettings.txt").write_bytes(build.read_bytes())
    script_guids = []
    for meta in sorted(scripts.rglob("*.cs.meta")):
        match = re.search(r"^guid:\s*([0-9a-f]{32})", meta.read_text(), re.MULTILINE)
        if match:
            source = Path(str(meta)[:-5])
            script_guids.append({"name": source.stem, "path": source.relative_to(PROJECT).as_posix(),
                                 "guid": match.group(1)})
    manifest = {"scenes": sources,
                "buildSettings": {"resource": "RobotAppScenes/EditorBuildSettings", "sha256": digest(build)},
                "scripts": script_guids}
    (resources / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    report_path = destination / "preparation-report.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    report["sceneWorkflowHarness"] = {
        "actualRuntimeSources": [{"path": path.relative_to(PROJECT).as_posix(), "sha256": digest(path)}
                                 for path in sorted(runtime)],
        "serializedSources": manifest,
        "transportBoundary": "The ROS factory throws." if options.stub_transport else "Tests keep actual service/connector components disabled and never call Connect or Start on ROS transport.",
        "testSelection": "Run only scene, welcome and lifecycle fixtures; older workflow tests intentionally use the separate throwing-service harness.",
        "validationPerformed": False,
    }
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Prepared actual scene navigation and serialized scene snapshots at {destination}")


if __name__ == "__main__":
    main()
