# Exact TCP A/B planning proposal

`TcpPlanning.patch` was applied on 2026-10-22 to the separate
`C:/Users/aca/source/UR16UnityControl` repository, on `codex/architecture-review`,
after the owner's explicit approval: “Apply the patch and build/test it.” Related
ROS API and project notes were updated. The reviewed patch is unchanged; source
hashes in `TcpPlanningSourceHashes.json` identify its original baseline. Builds
and tests use this Unity workspace's `Temp/TcpPlanningProposal`. No deployment,
application launch or hardware operation is included.

The ROS repository's [AGENTS.md](C:/Users/aca/source/UR16UnityControl/AGENTS.md)
requires: “Wait for approval before modifying files.” The owner's approval covers
this source/docs change and offline validation; it does not authorize launching
the robot stack or motion.

## User workflow

Capture exact TCP A and B, adjust their full orientations, and press **Plan**.
The backend plans from the current robot state through A to B and returns the
ghost trajectory for review. Planning performs no motion or vacuum IO. A separate
**Execute** action names the reviewed request and attempts the frozen solution
once, after the existing start-state, fresh-feedback and collision-scene checks.

This path supports `none`, `kenos` and `kenos_sl`, using the validated startup
model's `tcp` frame. It creates no tile, grasp orientation alternatives, contact
offset, vacuum stage or automatic return-to-start. Paths between the full TCP
goals use collision-aware OMPL planning; they are not prescribed straight lines.
Existing tile pick/place behavior retains its vacuum requirement and topics.

## Wire contract

| Direction | Topic/service | Type and behavior |
| --- | --- | --- |
| Probe | `/mtc_test_task_node/get_parameters` | Read `tcp_pose_planning_enabled`: read-only startup bool, default `true`. A false or missing capability blocks the Unity Plan action. |
| Plan | `/tcp_poses_plan_target` | `geometry_msgs/msg/PoseArray`: exactly two finite poses, A then B; `header.frame_id` must be `ur_base_link`. Quaternions must be finite and nonzero and are normalized without changing orientation. |
| Plan status | `/mtc/tcp_planning_status` | `std_msgs/msg/String` JSON with `overall_success`, `stages`, `message`, `request_id` and `phase`. |
| Preview | `/mtc/tcp_planned_trajectory` | `moveit_msgs/msg/RobotTrajectory`: selected A/B solution flattened for the ghost preview. Its joint-trajectory header stamp echoes the request token. |
| Execute | `/execute_tcp_plan` | `std_msgs/msg/String` containing the exact reviewed `request_id`. The backend checks mode and ID under the same mutex as single-use admission. |
| Execution result | `/mtc/tcp_execution_status` | `std_msgs/msg/String` JSON with `status`, `reason` and the incoming execute `request_id`. Status is `rejected`, `executing`, `succeeded` or `failed`. |

The plan request's nonzero, valid `header.stamp` is an **opaque request token**.
The ID is decimal `sec:nanosec` without padding. Generate a fresh token for every
plan request; never reuse one after editing targets. The backend uses timestamp
zero on actual MoveTo goals, so the token is never interpreted as a historical
TF time. No flange-to-TCP compensation is applied to the supplied poses.

Initial status is `overall_success=false, phase="planning"`; this is progress,
not a terminal failure. A successful request publishes its nonempty preview and
then `overall_success=true, phase="ready"`. Transport may deliver the two topics
in either order. Terminal failure uses `phase="failed"`; replacing a task with
the other mode publishes `phase="invalidated"` for the old mode. Unity requires
matching ready status **and** nonempty preview before offering Execute, and
ignores results for previous tokens. Invalidating a prior preview may also send
an empty trajectory; an empty trajectory does not establish a completed preview.

The shared MTC slot holds one immutable prepared plan. A new admitted tile or TCP
request replaces it. A wrong mode/ID cannot consume another prepared plan.
`/execute_task` remains tile-only and rejects TCP plans. TCP execution additionally
disables the executor's IO client and rejects any IO marker. Planning and
execution retain the existing callback mutex and PlanGate; execution eligibility
is consumed before executor construction and preflight, and every retry requires
fresh planning. This adds request matching, not shared robot ownership or
cancellation; another client can still move the arm or alter the scene.

## Verification

The applied ROS source package and `test_task` executable compile with GCC 13.3.0,
ROS Jazzy and installed MTC 0.1.4 in the existing `urmir-jazzy:latest` image.
Tests run in an ephemeral, network-disabled, read-only container, with only the
staged Unity `Temp` directory mounted writable and ROS logs redirected to `/tmp`.
No application node, launch file, controller, robot stack or hardware was started;
the parameter tests construct in-process nodes without an executor or parameter
services. No package installation or administrator rights were required.

- Six new TCP request/admission cases pass: complete full-pose preservation,
  frame/count validation, invalid second-pose rejection, quaternion sign/scale,
  token validation and mode/ID checks before one-shot gate consumption.
- All eight startup-parameter cases pass, including the two new immutable bool
  cases. Existing plan-validation, tool-model, PlanGate, executor-ordering and
  tool-definition suites pass.
- Seven of eight CTest targets pass. The unchanged legacy
  `SceneEffectsTest.SerializedPickPlaceEffectsUpdateTheLiveSceneInOrder` fails at
  `test_scene_effects.cpp:166`: MoveIt returns no tile/hand ACM entry after the
  scene sequence, while the test expects an explicit entry. The exact same
  failure was reproduced by compiling the untouched baseline against the same
  image. The other three scene-effect cases pass. It is not hidden or modified
  by this proposal.
- `git apply --check` passes against the preserved baseline. Actual ROS-TCP
  transport, planning reachability, physical poses and hardware execution remain
  untested. Unity's protocol checks separately cover progress, correlated
  preview/result ordering and stale-result rejection.

The applied source matches the reviewed staged source, and the ROS API/task/build
notes and project memory record the change. Deployment and robot operation remain
separate actions.
