"""Record the further 55.25 -> 53.25 mm correction against the approved Unity baseline.

Uses the actual generated catalog, retained URDF, and isolated Unity NUnit result.
It does not start ROS, open a robot transport, or overwrite historical 5 mm evidence.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from check_robot_frames import chain, compose, parse_joint
from verify_runtime_models import pose_error

PROJECT = Path(__file__).resolve().parents[2]
BASELINE = "f397551c119cfc80f76d04afb3881e0824486f42"
RESOURCES = "Assets/Resources/RobotModels/"


def git_bytes(reference, relative):
    return subprocess.check_output(["git", "-C", str(PROJECT), "show", f"{reference}:{relative}"])


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record(baseline, unity_results):
    before = json.loads(git_bytes(baseline, RESOURCES + "Models.json"))
    after = json.loads((PROJECT / RESOURCES / "Models.json").read_text())
    assert before["dependencies"] == after["dependencies"], "Upstream revisions changed"
    old_models = {model["tool"]: model for model in before["models"]}
    new_models = {model["tool"]: model for model in after["models"]}
    assert old_models.keys() == new_models.keys() == {"none", "kenos", "kenos_sl"}
    changed = unchanged = 0
    maximum_error = 0.0
    for tool, current in new_models.items():
        previous = old_models[tool]
        if tool != "kenos_sl":
            assert previous == current, f"{tool} model changed"
            assert git_bytes(baseline, RESOURCES + tool + ".urdf.txt") == (PROJECT / RESOURCES / (tool + ".urdf.txt")).read_bytes()
        else:
            expected = copy.deepcopy(previous["links"])
            for link in expected:
                if link["name"] == "kenos_sl_mount":
                    for cylinder in link["visuals"] + link["collisions"]:
                        assert cylinder["length"] == 0.05525 and cylinder["position"][2] == 0.027625
                        cylinder["length"] = 0.05325
                        cylinder["position"][2] = 0.026625
                if link["name"] == "kenos_sl_base":
                    assert link["position"] == [0.0, 0.0, 0.05525]
                    link["position"][2] = 0.05325
            assert expected == current["links"], "Unexpected local link/shape/rotation change"
        assert current["calibrationHash"] == previous["calibrationHash"]
        assert len(current["goldenCases"]) == len(previous["goldenCases"]) == 21
        root = ET.parse(PROJECT / RESOURCES / (tool + ".urdf.txt")).getroot()
        joints = [parse_joint(joint) for joint in root.findall("joint")]
        for old_case, new_case in zip(previous["goldenCases"], current["goldenCases"]):
            for key in ("name", "jointNames", "jointPositions"):
                assert old_case[key] == new_case[key]
            angles = dict(zip(new_case["jointNames"], new_case["jointPositions"]))
            mount = compose(chain(joints, "ur_base_link", "kenos_sl_mount"), angles) if tool == "kenos_sl" else None
            assert len(old_case["links"]) == len(new_case["links"])
            for old, new in zip(old_case["links"], new_case["links"]):
                assert old["name"] == new["name"]
                expected_position = list(old["position"])
                if tool == "kenos_sl" and new["name"] in ("kenos_sl_base", "tcp"):
                    expected_position = [old["position"][axis] - 0.002 * mount[axis][2] for axis in range(3)]
                    assert abs(math.dist(old["position"], new["position"]) - 0.002) < 1e-12
                    changed += 1
                else:
                    unchanged += 1
                error = math.dist(expected_position, new["position"])
                maximum_error = max(maximum_error, error)
                assert error < 1e-12 and pose_error(old, new)[1] < 1e-10
    assert (changed, unchanged) == (42, 861)
    assert git_bytes(baseline, RESOURCES + "KenosSLVisual.bytes") == (PROJECT / RESOURCES / "KenosSLVisual.bytes").read_bytes()
    # Git normalizes JSON line endings; compare the unmodified provenance content.
    assert json.loads(git_bytes(baseline, RESOURCES + "KenosSLVisualProvenance.json")) == json.loads((PROJECT / RESOURCES / "KenosSLVisualProvenance.json").read_text())
    historical = ("Tools/RobotFrames/tool_mount_validation_summary.json",
                  "Tools/RobotFrames/tool_mount_source_sync_validation_summary.json",
                  "Tools/RobotFrames/ToolMountValidation/source-default-parity-20261102.json")
    for relative in historical:
        assert json.loads(git_bytes(baseline, relative)) == json.loads((PROJECT / relative).read_text()), relative
    result = ET.parse(unity_results).getroot()
    assert result.attrib["result"] == "Passed" and int(result.attrib["failed"]) == 0
    assert int(result.attrib["total"]) > 0
    validated = PROJECT / "Library/ToolMount2mmValidation"
    files = [RESOURCES + "Models.json", RESOURCES + "kenos_sl.urdf.txt",
             "Assets/Tests/RobotFrames/RosRobotModelTests.cs", "Assets/Tests/RobotFrames/RosRobotRigPrefabTests.cs"]
    preparation = json.loads((validated / "model-only-preparation.json").read_text())
    files = sorted(set(files + preparation["keptSourceFiles"]))
    for relative in files:
        assert digest(PROJECT / relative) == digest(validated / relative), f"Validated copy differs: {relative}"
    report = {
        "baselineUnityCommit": baseline, "mountBeforeM": 0.05525, "mountAfterM": 0.05325,
        "additionalReductionM": 0.002, "axialFlangeToTcpBeforeM": 0.145, "axialFlangeToTcpAfterM": 0.143,
        "movedDistalLinkPoses": changed, "unchangedOtherLinkPoses": unchanged,
        "maximumExpectedPositionErrorM": maximum_error, "noneAndKenosModelsUnchanged": True,
        "slLocalToolGeometryAndTcpUnchanged": True, "cadMeshBytesUnchanged": True, "cadProvenanceContentUnchanged": True,
        "upstreamDependencies": after["dependencies"],
        "portableTests": {"passed": 17, "command": "python -B -m unittest discover -s Tools/RobotFrames -p test_model_export.py"},
        "unity": {"version": "6000.0.84f1", "filter": "RosRobotModelTests;RosRobotRigPrefabTests",
                  "total": int(result.attrib["total"]), "passed": int(result.attrib["passed"]),
                  "failed": int(result.attrib["failed"]), "resultXmlSha256": digest(unity_results),
                  "isolatedProject": "Library/ToolMount2mmValidation",
                  "fixtureScope": "Actual model, rig, rendering and support sources; unrelated UI/workflow copies excluded after the shared preparer omitted their dependencies",
                  "preparationReportSha256": digest(validated / "model-only-preparation.json")},
        "files": [{"path": relative, "sha256": digest(PROJECT / relative)} for relative in files],
        "historicalFiveMillimetreEvidencePreserved": True,
        "hardwareExercised": False, "rosNodesStarted": False, "deploymentPerformed": False,
        "limitations": ["No Android/player build or headset deployment", "Active ROS runtime must be rebuilt/restarted separately to consume the changed source defaults"]}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-ref", default=BASELINE)
    parser.add_argument("--unity-results", type=Path, default=PROJECT / "Library/ToolMount2mmValidation/results.xml")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("tool_mount_2mm_validation_summary.json"))
    args = parser.parse_args()
    report = record(args.baseline_ref, args.unity_results)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded exact 2 mm reduction: 42 distal poses moved, 861 unchanged; {report['unity']['passed']} Unity tests passed")


if __name__ == "__main__":
    main()
