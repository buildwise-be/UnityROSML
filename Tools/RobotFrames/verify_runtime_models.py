#!/usr/bin/env python3
"""Compare a Unity runtime FK dump with every expanded ROS golden link pose.

The dump uses {models:[{tool,cases:[{name,links:[{name,position,rotation}]}]}]}.
Both poses use ROS FLU, metres, XYZW and the ur_base_link-relative frame. A
position or orientation error, missing/extra tool/case/link, or invalid pose
fails verification. A nominal Unity model cannot pass by matching only its TCP.
"""

import argparse
import json
import math
from pathlib import Path
import sys


def indexed(records, label):
    result = {}
    for record in records:
        key = record["tool"] if label == "tool" else record["name"]
        if key in result:
            raise ValueError(f"Duplicate {label} {key}")
        result[key] = record
    return result


def pose_error(expected, actual):
    def vector(record, name, size):
        values = record[name]
        if len(values) != size or not all(math.isfinite(value) for value in values):
            raise ValueError(f"Invalid {name}: {values}")
        return values
    ep, ap = vector(expected, "position", 3), vector(actual, "position", 3)
    eq, aq = vector(expected, "rotation", 4), vector(actual, "rotation", 4)
    en = math.sqrt(sum(value * value for value in eq))
    an = math.sqrt(sum(value * value for value in aq))
    if en < 1e-12 or an < 1e-12 or abs(an - 1) > 0.001:
        raise ValueError("Invalid or nonunit runtime quaternion")
    # atan2 gives a stable small-angle difference; quaternion signs are equivalent.
    sign = 1 if sum(a * b for a, b in zip(eq, aq)) >= 0 else -1
    eq, aq = [v / en for v in eq], [sign * v / an for v in aq]
    chord = math.sqrt(sum((a - b) ** 2 for a, b in zip(eq, aq)))
    angle = math.degrees(4 * math.asin(min(1, chord / 2)))
    return math.dist(ep, ap), angle


def verify(expected, actual, position_tolerance_m=0.00001, angle_tolerance_degrees=0.001):
    expected_models, actual_models = indexed(expected["models"], "tool"), indexed(actual["models"], "tool")
    failures, total = [], 0
    maxima = {"positionErrorM": 0.0, "orientationErrorDegrees": 0.0}

    def same_keys(wanted, observed, scope):
        for key in sorted(set(wanted) - set(observed)):
            failures.append(f"{scope}: missing {key}")
        for key in sorted(set(observed) - set(wanted)):
            failures.append(f"{scope}: unexpected {key}")

    same_keys(expected_models, actual_models, "models")
    for tool in sorted(set(expected_models) & set(actual_models)):
        expected_cases = indexed(expected_models[tool]["goldenCases"], "case")
        actual_cases = indexed(actual_models[tool]["cases"], "case")
        same_keys(expected_cases, actual_cases, tool)
        for case in sorted(set(expected_cases) & set(actual_cases)):
            expected_links = indexed(expected_cases[case]["links"], "link")
            actual_links = indexed(actual_cases[case]["links"], "link")
            scope = f"{tool}/{case}"
            same_keys(expected_links, actual_links, scope)
            for name in sorted(set(expected_links) & set(actual_links)):
                total += 1
                try:
                    position, angle = pose_error(expected_links[name], actual_links[name])
                except (KeyError, TypeError, ValueError) as error:
                    failures.append(f"{scope}/{name}: {error}")
                    continue
                maxima["positionErrorM"] = max(maxima["positionErrorM"], position)
                maxima["orientationErrorDegrees"] = max(maxima["orientationErrorDegrees"], angle)
                if position > position_tolerance_m or angle > angle_tolerance_degrees:
                    failures.append(f"{scope}/{name}: {position * 1000:.6f} mm, {angle:.8f} deg")
    return {"passed": not failures, "verifiedLinkPoses": total, **maxima,
            "positionToleranceM": position_tolerance_m, "orientationToleranceDegrees": angle_tolerance_degrees,
            "failures": failures}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    project = Path(__file__).resolve().parents[2]
    parser.add_argument("--expected", type=Path, default=project / "Assets/Resources/RobotModels/Models.json")
    parser.add_argument("--actual", type=Path, required=True)
    parser.add_argument("--report", type=Path, default=Path(__file__).with_name("runtime_verification.json"))
    args = parser.parse_args()
    report = verify(json.loads(args.expected.read_text()), json.loads(args.actual.read_text()))
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
