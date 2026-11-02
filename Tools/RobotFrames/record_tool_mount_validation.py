#!/usr/bin/env python3
"""Record the 5 mm Kenos SL mount correction against the previous committed model."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

from verify_runtime_models import pose_error


PROJECT = Path(__file__).resolve().parents[2]
CATALOG = Path("Assets/Resources/RobotModels/Models.json")
ISOLATED = PROJECT / "Library/PointRegistrationValidation"
EVIDENCE = PROJECT / "Library/ToolMountCorrection"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def indexed(records, key):
    return {record[key]: record for record in records}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-root", type=Path, required=True)
    parser.add_argument("--baseline-ref", default="38c1f3b")
    args = parser.parse_args()
    before_bytes = subprocess.check_output(["git", "show", args.baseline_ref + ":" + CATALOG.as_posix()], cwd=PROJECT)
    before = json.loads(before_bytes)
    after = json.loads((PROJECT / CATALOG).read_text(encoding="utf-8-sig"))
    assert before["dependencies"] == after["dependencies"], "Upstream dependency revisions changed"
    assert before["unavailableMeshUris"] == after["unavailableMeshUris"]
    old_models, new_models = indexed(before["models"], "tool"), indexed(after["models"], "tool")
    assert old_models.keys() == new_models.keys()
    changed = unchanged = 0
    for tool, current in new_models.items():
        previous = old_models[tool]
        assert previous["calibrationHash"] == current["calibrationHash"]
        if tool != "kenos_sl":
            assert previous == current, tool + " model changed"
        old_links, new_links = indexed(previous["links"], "name"), indexed(current["links"], "name")
        for name, link in new_links.items():
            old = old_links[name]
            if tool == "kenos_sl" and name == "kenos_sl_mount":
                for category in ("visuals", "collisions"):
                    assert len(link[category]) == 1
                    cylinder = link[category][0]
                    assert cylinder["kind"] == "cylinder" and cylinder["length"] == 0.05525
                    assert cylinder["position"] == [0.0, 0.0, 0.027625] and cylinder["radius"] == 0.05
            elif tool == "kenos_sl" and name == "kenos_sl_base":
                assert link["position"] == [0.0, 0.0, 0.05525]
                assert link["visuals"] == old["visuals"] and link["collisions"] == old["collisions"]
            else:
                assert link == old, tool + "/" + name + " local geometry or transform changed"
        for sample, old_sample in zip(current["goldenCases"], previous["goldenCases"]):
            assert sample["name"] == old_sample["name"] and sample["jointPositions"] == old_sample["jointPositions"]
            old_poses = indexed(old_sample["links"], "name")
            x, y, z, w = old_poses.get("kenos_sl_mount", {"rotation": [0, 0, 0, 1]})["rotation"]
            outward = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
            for pose in sample["links"]:
                old_pose = old_poses[pose["name"]]
                distance, angle = pose_error(old_pose, pose)
                assert angle < 1e-10
                if tool == "kenos_sl" and pose["name"] in ("kenos_sl_base", "tcp"):
                    delta = [pose["position"][axis] - old_pose["position"][axis] for axis in range(3)]
                    assert math.isclose(distance, 0.005, abs_tol=1e-12)
                    assert all(abs(delta[axis] + outward[axis] * 0.005) < 1e-12 for axis in range(3))
                    changed += 1
                else:
                    assert distance < 1e-12
                    unchanged += 1
    assert changed == 42 and unchanged == 861

    results = ISOLATED / "tool-mount-results.xml"
    run = ET.parse(results).getroot()
    assert run.get("result") == "Passed" and int(run.get("failed")) == 0 and int(run.get("skipped")) == 0
    ros_results = EVIDENCE / "ros-source-model-results.xml"
    cases = list(ET.parse(ros_results).getroot().iter("testcase"))
    assert len(cases) == 11 and all(case.find("failure") is None and case.find("error") is None for case in cases)
    ros_run = json.loads((EVIDENCE / "ros-source-model-test-summary.json").read_text())
    assert ros_run["pytestExitCode"] == 0 and ros_run["networkMode"] == "none" and not ros_run["rosNodeActionsExecuted"]

    paths = [CATALOG] + [Path("Assets/Resources/RobotModels") / (tool + ".urdf.txt") for tool in new_models]
    paths += [Path(path) for path in ("Assets/Tests/RobotFrames/RosRobotModelTests.cs",
              "Assets/Tests/RobotFrames/RosRobotRigPrefabTests.cs", "Assets/Resources/RobotModels/KenosSLVisual.bytes",
              "Assets/Resources/RobotModels/KenosSLVisualProvenance.json")]
    hashes = []
    for path in paths:
        assert digest(PROJECT / path) == digest(ISOLATED / path), "Tested copy differs: " + str(path)
        hashes.append({"path": path.as_posix(), "sha256": digest(PROJECT / path)})
    for name in ("KenosSLVisual.bytes", "KenosSLVisualProvenance.json"):
        relative = "Assets/Resources/RobotModels/" + name
        prior = subprocess.check_output(["git", "show", args.baseline_ref + ":" + relative], cwd=PROJECT)
        current = (PROJECT / relative).read_bytes()
        if name.endswith(".json"):
            # Git stores LF; this standard Windows checkout uses CRLF.
            assert prior.replace(b"\r\n", b"\n") == current.replace(b"\r\n", b"\n"), "CAD provenance changed"
        else:
            assert prior == current, "CAD geometry changed"
    ros_hashes = []
    for source in after["sourceFiles"]:
        if source["path"].startswith("ros/"):
            relative = source["path"][4:]
            assert digest(args.ros_root / relative) == source["sha256"], "ROS source changed after export: " + relative
            ros_hashes.append({"path": relative, "sha256": source["sha256"]})

    output = PROJECT / "Tools/RobotFrames/ToolMountValidation"
    output.mkdir(exist_ok=True)
    for name in ("ros-source-model-results.xml", "ros-source-model-test-summary.json", "run_ros_source_model_tests.py"):
        shutil.copy2(EVIDENCE / name, output / name)
    summary = {
        "schemaVersion": 1, "completedUtc": datetime.now(timezone.utc).isoformat(), "tool": "kenos_sl",
        "baselineCommit": args.baseline_ref, "baselineCatalogSha256": hashlib.sha256(before_bytes).hexdigest(),
        "mountLengthM": {"before": 0.06025, "after": 0.05525},
        "axialFlangeToTcpM": {"before": 0.15, "after": 0.145}, "toolLocalTcpRosM": [0.034, 0, 0.08975],
        "baselineComparison": {"changedDistalPoses": changed, "unchangedPoses": unchanged, "orientationsUnchanged": True,
                               "noneAndFlexiGripUnchanged": True, "cadBytesAndProvenanceUnchanged": True},
        "unityTests": {key: int(run.get(key)) for key in ("total", "passed", "failed", "skipped")},
        "unityTestResults": {"path": results.relative_to(PROJECT).as_posix(), "sha256": digest(results)},
        "rosSourceModelTests": {"passed": len(cases), "failed": 0, "result": "ToolMountValidation/ros-source-model-results.xml",
                                "sha256": digest(ros_results), "networkMode": "none", "rosNodeActionsExecuted": False},
        "validatedUnitySources": hashes, "validatedRosSources": ros_hashes, "dependencies": after["dependencies"],
        "limits": ["This is the owner's requested empirical 5 mm mounting correction, not a measured full tool calibration.",
                   "ROS source and Unity offline models are corrected; no running stack was deployed or restarted.",
                   "The corrected live ROS descriptions require rebuilding/restarting all model consumers together."],
        "hardwareExercised": False, "robotCommandsSent": False, "deployed": False,
    }
    (PROJECT / "Tools/RobotFrames/tool_mount_validation_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {run.get('passed')} Unity checks, {len(cases)} ROS model cases and {changed + unchanged} baseline poses")


if __name__ == "__main__":
    main()
