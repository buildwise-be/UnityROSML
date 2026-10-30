# MTC execution precision fix — verified and deployed

Applied to `C:/Users/aca/source/UR16UnityControl` after the owner's explicit
approval, “Apply and build/test”. The full ROS package and application compile;
all 14 plan-validation cases pass. The separately approved Nordbo deployment and
MTC-only restart completed on **2026-10-28**. See the deployment evidence in
[`../ExecutePrecisionValidation_20261028/`](../ExecutePrecisionValidation_20261028/).

The 2026-10-23 MTC plan reached `/execute_task`, but ROS rejected execution
with `world collision geometry or octomap changed`. The retained solution and a
read-only `GetPlanningScene` response have identical world data except for three
object quaternion coefficients, differing by 3.47–6.94 × 10^-18. The old
validator compared those messages exactly.

Evidence: [diagnosis](../mtc_execute_scene_diagnosis_20261023.json) and
[retained solution/current scene](../mtc_execute_scene_live_20261023.json).

## Change

The [patch](execute-world-quaternion-roundoff.patch) changes only:

- `ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp`
- `ros_ws/src/ur16e_mtc_gpio_stage/test/test_plan_validation.cpp`

Before the existing exact world comparison, align matching world-object unit
quaternion representations within **1e-12 per component**, on the validator's
local comparison copy. Both quaternions must have finite components and squared
norm within 1e-12 of one. Equivalent `q` and `-q` rotations use the same bound.
Apply this to each object's own pose and primitive, mesh, plane and subframe
poses. Existing scene normalization sorts corresponding IDs and subframes first.
The final exact scene comparison uses this same aligned copy.

Positions, shape dimensions and mesh data, object IDs, coordinate frames,
octomap contents and origin, attachments, referenced fixed transforms, collision
permissions, joint feedback checks, padding and scaling keep their existing
comparisons. The observed empty octomap is identical, so this patch does not
change octomap comparison. It does not change plan consumption or execution.

The accompanying five regression tests cover the three recorded coefficients,
input immutability, all five world-pose locations, quaternion sign equivalence,
finite/unit validation, the 1e-12 bound, and rejection of actual changes. Tiny
position/dimension changes, octomap changes, attachment rotation and referenced
fixed-frame rotation continue to reject.

## Approval and validation

The patch is applied on `codex/architecture-review`, based on HEAD
`040e1eb1df863938324d791a5e1927dea6efc1aa`. The copied files and final patch match
the applied source after Windows line-ending normalization. Exact source bytes
used by the isolated build match all 29 files of the actual package. Source,
proposal, applied and tested SHA-256 hashes are recorded in
[manifest.json](manifest.json) and the [validation summary](Validation/summary.json).

[The ROS checkout's AGENTS.md](C:/Users/aca/source/UR16UnityControl/AGENTS.md)
requires: “Wait for approval before modifying files.” The owner's approval covers
the applied source, API/project notes and offline package build/tests. That initial
approval excluded deployment and restarts. The owner separately approved
“Deploy fix and restart MTC only” on 2026-10-28. The driver, MoveIt and ROS–Unity
endpoint remained running; no Plan, Execute or IO command was sent.
No commit or push was performed.

Validation used existing local WSL image `urmir-jazzy:latest`, SHA256
`752aa09527488fce5baaa35ebbff1aad52c6e82e72a302db4f14ded79f2f615b`, with GCC 13.3.0,
ROS Jazzy and MTC 0.1.4. The dedicated container had networking disabled, a read-only
root filesystem and source mounts, temporary build outputs and separate evidence.
No installation or administrator rights were needed. The application executable
was compiled but not started; no planning, Execute request, driver or IO was run.

- The [full build](Validation/build.log) passes, including `test_task`.
- All **14 plan-validation cases** pass, including the five new regressions.
- **Nine of ten CTest targets** pass. The sole unchanged failure is
  `SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder`,
  `test_scene_effects.cpp:166`: `getEntry("tile", "hand", permission)` returns
  `false` while the assertion expects `true`.
  The [baseline reproduction](Validation/baseline-scene-effects.log) confirms the
  same failure using the unchanged scene-effect implementation/test and original
  HEAD validator header. This correction does not modify or exclude that test.
- The [recorded quaternion regression](Validation/baseline-recorded.log) fails
  with the unchanged HEAD header, SHA256
  `f301530268a84578af0494d8a5f6ece06499770cbc690a670e0eacb78f0c72c8`,
  and passes with the applied validator. Baseline compilation uses the final new
  regression test source; the header supplies the production before/after change.
- The [complete captured-scene replay](Validation/captured-scene-replay.log)
  serializes both saved `PlanningScene` messages through actual ROS types and
  checks every production validator guard. The patched header returns **PASS**;
  the untouched HEAD header returns `world collision geometry or octomap changed`.
  Both use the same saved snapshot. This check initializes no ROS nodes and makes
  no ROS graph calls. See [the result](Validation/captured-scene-result.json).

The first run exposed a fixture mistake: ROS `Quaternion()` defaults to identity,
so the intended zero-quaternion case now explicitly sets `w=0`. Initial results
are retained in `Validation/initial-fixture-run/`; final results are in
[ctest.log](Validation/ctest.log) and [test_results](Validation/test_results/ur16e_mtc_gpio_stage/test_plan_validation.gtest.xml).
The production fix is unchanged from the reviewed proposal. The final patch and
manifest include this fixture correction. Temporary containers were removed.

The ROS API/execution contract, build/testing notes and operational project memory
record the source change and these verification limits. The 2026-10-28 deployment
records include live process, exact configuration/model and fresh joint-feedback
checks. Actual hardware execution remains untested by this work.
