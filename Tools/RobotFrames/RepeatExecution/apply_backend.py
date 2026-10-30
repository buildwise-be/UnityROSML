#!/usr/bin/env python3
"""Apply only the reviewed repeat-source overlay after checking all original hashes."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parent
ROS = Path("C:/Users/aca/source/UR16UnityControl")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def add_before_refresh(relative, heading, paragraph):
    target = ROS / relative
    original = target.read_text(encoding="utf-8-sig")
    if heading in original:
        return
    index = original.find("\n## Refresh\n")
    if index < 0:
        updated = original.rstrip() + "\n\n" + paragraph.strip() + "\n"
    else:
        updated = original[:index].rstrip() + "\n\n" + paragraph.strip() + "\n" + original[index:]
    target.write_text(updated, encoding="utf-8")


def prepend_memory(relative, paragraph):
    target = ROS / relative
    original = target.read_text(encoding="utf-8-sig")
    if paragraph.strip() in original:
        return
    first, rest = original.split("\n", 1)
    target.write_text(first + "\n\n" + paragraph.strip() + "\n" + rest, encoding="utf-8")


def main():
    manifest = json.loads((ROOT / "manifest.json").read_text(encoding="utf-8"))
    for change in manifest["changes"]:
        if digest(ROS / change["path"]) != change["beforeSha256"]:
            raise ValueError("Original source changed; refusing to overwrite: " + change["path"])
        if digest(ROOT / change["path"]) != change["afterSha256"]:
            raise ValueError("Reviewed source overlay changed: " + change["path"])
    for change in manifest["changes"]:
        target = ROS / change["path"]
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / change["path"], target)

    add_before_refresh("docs/wiki/API.md", "## Explicit repeat of a completed reviewed plan", '''## Explicit repeat of a completed reviewed plan

The additive `/repeat_reviewed_plan` topic accepts a `std_msgs/String` JSON
object containing exactly `plan_id` and `attempt_id`. Both are nonempty bounded
ASCII tokens; each explicit click supplies a new GUID attempt ID. Successful
planning/ready statuses publish an opaque per-plan token. Execution statuses on
the corresponding existing MTC/TCP topic include `plan_id`, `attempt_id`, and
`repeatable`. TCP retains its separate original `request_id`.

Only a confirmed successful execution retains its immutable solution and emits
`repeatable:true`. Original `execute_task` and `execute_tcp_plan` actions remain
spent. The new action reruns unchanged start-joint, feedback-age, tool/model and
full-scene checks. Duplicates of a consumed attempt never execute or overwrite an
active outcome. No response means an unknown outcome, never permission to retry
automatically. Busy repeat callbacks also do nothing rather than publishing a
terminal result that could race the same attempt's active execution.

Scene setup, replacement planning, failure or invalidation revokes the saved
repeat. An admitted repeat failure requires a new plan. Return-to-start joint
feedback alone does not restore objects or the scene; MTC's placed object must
also match the original recorded start scene. The endpoint performs no scene
reset or automatic robot return. Sources: `src/test_task.cpp`,
`include/ur16e_mtc_gpio_stage/repeat_execution.hpp`, and
`test/test_repeat_execution.cpp` under `ros_ws/src/ur16e_mtc_gpio_stage`.
This 2026-10-29 source interface is locally compiled/tested; transport and
hardware execution remain unverified.''')

    add_before_refresh("docs/wiki/TASK_EXECUTION.md", "## Identified repeat admission", '''## Identified repeat admission

A confirmed successful execution may retain its immutable `PreparedPlan` in
`completed_plan_`; the original `prepared_plan_` is still consumed. The separate
identified repeat action is admitted only from `PlanGate::Idle`, after matching
the retained plan token and recording a new attempt ID. It moves the completed
plan into the callback before constructing execution clients or checking state.
The same `runPreparedPlan` function and original full scene/joint validator run
for both first and repeated attempts. Failure revokes repeat eligibility;
confirmed success retains the same plan and permanent attempt history again.

The current plan remembers up to 1,024 attempt IDs and never evicts them. At the
limit, a new plan is required. Replacement planning and scene setup discard the
retained plan and its token. A busy repeat callback performs no operation and
publishes no terminal status, because it can be a duplicate of the running
attempt. The legacy Bool and exact-TCP token endpoints still cannot repeat a
consumed plan. The start scene remains original: no moved/attached object or
collision permission is silently restored just because arm joints returned.
See the [wire contract](API.md#explicit-repeat-of-a-completed-reviewed-plan).''')

    prepend_memory("docs/ai-context/CURRENT_STATE.md", '''- Owner-requested explicit reviewed-plan repeat is implemented in local source
  (2026-10-29), together with matching Unity changes. Legacy Execute stays
  single-use; a new plan/attempt-ID endpoint retains only confirmed-success
  solutions and reruns all original joint/scene/tool/model checks. Complete MTC
  compilation passed in a network-disabled local container; 10/11 CTest targets
  pass, including 37 new protocol/admission checks. The unchanged known
  `test_scene_effects.cpp:166` assertion remains. No ROS application, deployment,
  stack restart, robot motion or IO operation was performed.''')
    prepend_memory("docs/ai-context/DECISIONS.md", '''## 2026-10-29: Explicitly repeat a completed reviewed path

- Owner asked to re-execute a planned path once the robot returned to its initial
  position. Add a separate identified repeat action rather than rearming the
  existing anonymous Bool command or automatically resending an execution.
- Keep the original immutable solution only after confirmed success. Require a
  fresh GUID per explicit repeat; retain consumed IDs without eviction for the
  current plan and fail closed at the bounded attempt limit.
- Recheck the original full planning scene and fresh unwrapped joint state,
  including tool/model configuration. Returning the arm does not authorize
  silently resetting an object, attachment, collision permission or vacuum IO.
- Source changes and isolated offline validation are authorized by the feature
  request; deployment, live-stack restart and physical execution are separate.''')
    prepend_memory("docs/ai-context/KNOWN_ISSUES.md", '''- Identified repeat execution is local source only until the updated MTC node
  is built/deployed by an authorized operator. No actual ROS-TCP repeat request
  or hardware operation was exercised. A successful MTC return stage restores
  arm joints but leaves its modeled object at place, so repeat still rejects
  unless the original scene is restored too. Preflight or execution failure
  consumes repeat eligibility and requires a new plan. Busy/duplicate repeat
  callbacks do not publish terminal status; clients must keep unknown outcomes
  blocked rather than retrying automatically.''')
    output = {"sourceApplied": True, "files": [item["path"] for item in manifest["changes"]],
              "deployment": False, "hardware": False}
    (ROOT / "application_result.json").write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print("Applied five guarded backend source/test files; preserved prior changes; no deployment or hardware operation.")


if __name__ == "__main__":
    main()
