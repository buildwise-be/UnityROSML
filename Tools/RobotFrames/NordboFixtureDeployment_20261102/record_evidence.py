#!/usr/bin/env python3
"""Index deployment evidence already collected by the authorized operator.

This script only reads local evidence and writes summary.json. It never opens an
SSH/ROS connection, builds an image, changes a container or signals a process.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


MANIFESTS = ("staging.json", "build-result.json", "test-result.json",
             "activation-result.json", "final-verification.json")
PACKAGES = ("kenos_flexigrip", "mir_description", "ur16e_bringup", "ur16e_commander",
            "ur16e_environment", "ur16e_interfaces", "ur16e_mir250_control",
            "ur16e_mir250_description", "ur16e_mir250_moveit_config", "ur16e_mtc_gpio_stage")
FORBIDDEN_COMMANDS = ("robot_motion_requested", "execution_requests_sent", "io_commands_sent")


def fingerprint(path, root):
    return {"path": path.relative_to(root).as_posix(), "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--outcome", choices=("staged", "verified_deployed", "blocked", "rolled_back"), required=True)
    args = parser.parse_args()
    root = args.directory.resolve()
    documents = {name: json.loads((root / name).read_text(encoding="utf-8-sig"))
                 for name in MANIFESTS if (root / name).is_file()}
    missing = sorted(set(MANIFESTS) - documents.keys())
    if args.outcome == "verified_deployed":
        if missing:
            raise ValueError("Deployment evidence is incomplete: " + ", ".join(missing))
        staging = documents["staging.json"]
        if not (staging.get("oldImage") and staging.get("sourceArchiveSha256")
                and staging.get("liveChanged") is False and staging.get("robotCommands") is False):
            raise ValueError("A guarded staging record is required")
        build = documents["build-result.json"]
        if build.get("exitCode") != 0 or not build.get("image") or build.get("network") != "none":
            raise ValueError("A successful network-disabled image build is required")
        for name in ("test-result.json", "activation-result.json", "final-verification.json"):
            if documents[name].get("success") is not True:
                raise ValueError("The operator must explicitly verify success in " + name)
        built = build.get("packages", [])
        if set(built) != set(PACKAGES):
            raise ValueError("The build must identify all ten local application packages")
        tests = documents["test-result.json"]
        if not tests.get("reports") or tests.get("unexpectedFailures") or tests.get("network") != "none":
            raise ValueError("Record all test results and reject unexpected failures")
        replays = tests.get("replays", [])
        if len(replays) != 2 or any(item.get("exitCode") != 0 or item.get("result", {}).get("passed") is not True
                                    for item in replays):
            raise ValueError("Both isolated fixture replays must pass")
        final = documents["final-verification.json"]
        for flag in FORBIDDEN_COMMANDS:
            if final.get(flag) is not False:
                raise ValueError("Final evidence must explicitly record " + flag + "=false")
        if final.get("production_plan_requests_sent") is not False:
            raise ValueError("Synthetic isolated replay must remain separate from production planning")

    files = sorted(path for path in root.rglob("*") if path.is_file()
                   and path.name != "summary.json" and "__pycache__" not in path.parts)
    supporting = {name: json.loads((root / name).read_text(encoding="utf-8-sig"))
                  for name in ("source-sync.json", "python-interface-verification.json", "original-checkout-preservation.json")
                  if (root / name).is_file()}
    first_path = root / "test-result-first.json"
    first = json.loads(first_path.read_text(encoding="utf-8-sig")) if first_path.is_file() else None
    summary = {
        "schemaVersion": 1, "recordedUtc": datetime.now(timezone.utc).isoformat(),
        "operatorRecordedOutcome": args.outcome,
        "recordingScope": "Local indexing of collected evidence; no remote or ROS operations are performed by this recorder",
        "expectedApplicationPackages": list(PACKAGES),
        "reportedApplicationPackages": documents.get("build-result.json", {}).get("packages", []),
        "reportedTestCases": sum(item.get("tests", 0) for item in documents.get("test-result.json", {}).get("reports", [])),
        "reportedFailedCases": sum(item.get("failed", 0) for item in documents.get("test-result.json", {}).get("reports", [])),
        "missingManifests": missing,
        "manifests": documents,
        "supportingManifests": supporting,
        "initialCandidateTests": None if first is None else {
            "evidence": fingerprint(first_path, root), "success": first.get("success"),
            "reportedCases": sum(item.get("tests", 0) for item in first.get("reports", [])),
            "failedCases": sum(item.get("failed", 0) for item in first.get("reports", [])),
            "supersededByFinalCandidate": args.outcome == "verified_deployed",
            "note": "The first candidate exposed launch-test helper assumptions; the test-only correction was rebuilt and the final candidate revalidated. Original failures remain recorded separately."},
        "evidenceFiles": [fingerprint(path, root) for path in files],
        "limits": ["Synthetic fixture replay is plan-only in a separate network-disabled container; it does not prove physical reachability or fixture behavior.",
                   "A successful deployment does not validate robot motion, vacuum wiring, physical calibration or headset operation.",
                   "Private environment and raw parameter backups stay on Nordbo and must not be added to this public evidence directory."],
    }
    (root / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"Recorded {args.outcome}: {len(documents)} manifests and {len(files)} evidence files")


if __name__ == "__main__":
    main()
