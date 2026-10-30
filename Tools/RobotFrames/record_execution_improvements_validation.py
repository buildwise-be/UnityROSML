#!/usr/bin/env python3
"""Record tested registration feedback, transparency and repeat-execution sources."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[2]
ISOLATED = PROJECT / "Library/PointRegistrationValidation"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def imported_material_text(path):
    # Unity's URP importer fills these disabled passes on import. Preserve all
    # other material fields and disabled passes when checking the tested input.
    def normalize(match):
        passes = [line[4:] for line in match.group(0).splitlines()[1:]
                  if line[4:] not in ("MOTIONVECTORS", "SHADOWCASTER")]
        return "  disabledShaderPasses:" + ("\n" + "\n".join("  - " + value for value in passes) if passes else " []")
    value, count = re.subn(r"^  disabledShaderPasses:(?: \[\])?(?:\n  - [^\n]+)*", normalize,
                           path.read_text(encoding="utf-8-sig"), flags=re.MULTILINE)
    if count != 1:
        raise ValueError("Expected one material shader-pass block: " + str(path))
    return value


def main():
    results = ISOLATED / "execution-results.xml"
    run = ET.parse(results).getroot()
    if run.get("result") != "Passed" or int(run.get("failed", "1")) != 0:
        raise ValueError("A complete passing execution-improvements run is required")
    paths = []
    for folder in ("Assets/Scripts/V3", "Assets/Tests/RobotFrames"):
        for source in (PROJECT / folder).rglob("*.cs"):
            relative = source.relative_to(PROJECT)
            if (ISOLATED / relative).is_file():
                paths.append(relative.as_posix())
    paths += ["Assets/Resources/RobotModels/Models.json", "Assets/Resources/RobotTargetMarkers/Opaque.mat",
              "Assets/Resources/RobotTargetMarkers/Transparent.mat", "Assets/Prefabs/ur16e_kenos/ur16e_kenos.prefab"]
    hashes = []
    for relative in sorted(set(paths)):
        source, tested = PROJECT / relative, ISOLATED / relative
        record = {"path": relative, "sha256": digest(source)}
        if digest(source) != digest(tested):
            if source.suffix != ".mat" or imported_material_text(source) != imported_material_text(tested):
                raise ValueError("Current source differs from tested copy: " + relative)
            record["testedSha256"] = digest(tested)
            record["importNormalization"] = "URP material import adds disabled MOTIONVECTORS/SHADOWCASTER passes; all other fields match."
        hashes.append(record)
    compile_path = PROJECT / "Tools/RobotFrames/runtime_compile_summary.json"
    compile_summary = json.loads(compile_path.read_text(encoding="utf-8-sig"))
    for source in compile_summary["validatedSources"]:
        if digest(PROJECT / source["path"]) != source["sha256"]:
            raise ValueError("Runtime compilation is stale: " + source["path"])
    backend_path = PROJECT / "Tools/RobotFrames/RepeatExecution/validation_summary.json"
    backend = json.loads(backend_path.read_text(encoding="utf-8"))
    summary = {
        "schemaVersion": 1,
        "completedUtc": datetime.now(timezone.utc).isoformat(),
        "scope": "Captured registration contact spheres, execution transparency, and explicit start-guarded repeats for MTC and TCP.",
        "unityVersion": "6000.0.84f1",
        "unityTests": {key: int(run.get(key, "0")) for key in ("total", "passed", "failed", "skipped")},
        "fixtures": sorted({case.get("classname") for case in run.iter("test-case")}),
        "testResults": {"path": str(results), "sha256": digest(results)},
        "validatedSources": hashes,
        "runtimeCompile": {"sourceCount": compile_summary["helperSourceCount"] + compile_summary["mainSourceCount"],
                           "errors": compile_summary["errorCount"], "summarySha256": digest(compile_path)},
        "backend": {"summary": "RepeatExecution/validation_summary.json", "sha256": digest(backend_path),
                    "packageCompilation": backend["packageCompilation"], "protocolChecks": backend["portableProtocolChecksPassed"],
                    "legacyPlanGateChecks": backend["legacyPlanGateCasesPassed"], "ctest": backend["ctest"]},
        "limits": [
            "ROS transport and physical headset/robot operation were not exercised. The Unity harness throws before obtaining ROS transport.",
            "Only original successful server-retained plans can repeat; all repeats preserve full backend preflight and use a new explicit attempt id.",
            "Returning joints alone does not restore moved objects, attachments or planning-scene state.",
            "The local ROS source was updated and compiled offline; no running stack was deployed, restarted or operated.",
        ],
        "hardwareExercised": False,
        "robotCommandsSent": False,
        "deployed": False,
    }
    output = PROJECT / "Tools/RobotFrames/execution_improvements_validation_summary.json"
    output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {summary['unityTests']['passed']} passing Unity checks and {len(hashes)} matching source/asset hashes")


if __name__ == "__main__":
    main()
