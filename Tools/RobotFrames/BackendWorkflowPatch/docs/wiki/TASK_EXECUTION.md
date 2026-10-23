# Task planning and execution

`ur16e_mtc_gpio_stage` builds tile pick/place or exact TCP A/B tasks with MoveIt
Task Constructor (MTC), publishes a preview, and admits one execution attempt for a
validated, immutable prepared solution. Its executable is named `test_task`; the node is
`mtc_test_task_node`. Despite the executable name, this is the application
entry point. Its CMake file also registers separate executor regression tests.

Sources: [task implementation](../../ros_ws/src/ur16e_mtc_gpio_stage/src/test_task.cpp),
[package build](../../ros_ws/src/ur16e_mtc_gpio_stage/CMakeLists.txt),
[launch](../../ros_ws/src/ur16e_mtc_gpio_stage/launch/mtc_commander.launch.py).

## Configuration and task inputs

The launch exposes `tool` (`none`, `kenos` or `kenos_sl`, default `none`), `ik_frame`
(default `tcp`, required to match the selected tool definition), `log_level`, `ur_type` (default `ur16e`), and
`kinematics_params_file` (default
`ur16e_mir250_control/config/ur16e_calibration.yaml`). It supplies MoveIt
model, semantics, kinematics, joint limits, and OMPL pipeline parameters.
The URDF builder explicitly receives `tool`, `ur_type`, and
`kinematics_params`; the latter takes the `kinematics_params_file` launch
argument. All five `kenos_sl_*` mounting/TCP arguments are also passed to the
URDF builder; see [tool parameters](ROBOT_AND_TOOLS.md). The SRDF builder separately receives `tool`. The
[description selected by `.setup_assistant`](../../ros_ws/src/ur16e_mir250_moveit_config/.setup_assistant)
is [`mir250_ur16e.urdf.xacro`](../../ros_ws/src/ur16e_mir250_description/urdf/mir250_ur16e.urdf.xacro),
whose Kenos/nominal-kinematics defaults are overridden by those launch
mappings. The same `tool` launch substitution is also passed as a typed string
parameter to the task node, so model generation and task eligibility share the
startup selection.

The [main bringup](../../ros_ws/src/ur16e_bringup/launch/ur16e_start.launch.py)
forwards the same selected tool, arm type, arm calibration file and Safe&Light mounting/TCP values to robot
control, MoveGroup, and this task launch. An independent task launch must
use matching values explicitly if robot control uses nondefault inputs.
The calibration default is the stored UR16e calibration even if a different
`ur_type` is selected; it is not selected automatically by robot type.
These explicit inputs align tool geometry and arm calibration but do not
unify every hardware-description setting; see
[robot model generation and configuration boundaries](ROBOT_AND_TOOLS.md).

The MTC launch also exposes two positive, finite execution-check parameters:

| Argument | Default | Meaning |
| --- | --- | --- |
| `execution_start_tolerance` | `0.01` | Maximum absolute difference from each planned start joint position, in radians for this UR arm; angles are not wrapped modulo 2π. |
| `execution_joint_state_max_age` | `1.0` | Maximum seconds since joint feedback was **received**, measured with a steady clock. This is not the message header timestamp's age. |

These thresholds are read at node startup; changing ROS parameters later does
not update the cached values. Robot descriptions must be nonempty parameters;
there is no description-topic fallback. The node snapshots URDF/SRDF at startup
and rejects planning after either changes, requiring a task-node restart with
consistent startup configuration. It loads and validates the robot model once,
before creating command subscriptions; new tasks reuse that startup model.

`tool` and `ik_frame` are read-only startup strings. The node resolves an immutable
[tool definition](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/tool_definition.hpp)
with a stable `tcp` and an optional vacuum capability. `none` has no vacuum
capability. `kenos` and `kenos_sl` each supply their collision group/contact links and IO bindings
used by both planning stages and execution. Unknown tools, mismatched model
content, missing contact collision geometry, invalid fixed chains/groups, or an `ik_frame` different from the
definition's `tcp` stop startup before command subscriptions exist. A different
IK frame requires a matching tool definition; it is no longer an arbitrary
launch override. See [the exact model contract](ROBOT_AND_TOOLS.md). These checks
validate topology and collision-geometry presence, not mesh quality or physical tool identity.

With `tool=none`, the MTC node still accepts `/scene_setup_box`. Pick/place
requests and legacy `true` tile-execution requests report `rejected` before changing the
scene, constructing task stages, creating an executor or issuing IO. They
invalidate readiness, discard the cached task/plan, clear the retained preview,
and publish `overall_success=false`. `/execute_task=false` remains a no-op.
This leaves basic TCP motion and scene setup available without a vacuum tool.

The task code directly names arm group `ur_arm`, object ID `object`, and the six
`ur_*` arm joints. Tile
dimensions are process globals: 0.1 m square, 0.004 m thick; the final approach
distance is 0.1 m. These values are not ROS parameters or message fields.

[`PickPlacePoses.msg`](../../ros_ws/src/ur16e_interfaces/msg/PickPlacePoses.msg)
contains two `geometry_msgs/PoseStamped` fields, `pick_pose` and `place_pose`.
The implementation treats their positions as tile top-surface centers. It
subtracts half the tile thickness from the reference frame's Z coordinate
to obtain each object-center position. This is a reference-frame Z offset;
it is not rotated by the supplied orientation. The message itself does not
document this convention or include tile dimensions or an object ID.

## Planning flow

After checking the mounted tool's vacuum capability, an admitted
`pick_place_poses_target` callback performs these steps:

1. Enter `Planning`, discard the previous prepared plan and publish an empty
   preview before replacing the task or changing the scene.
2. Require configured URDF/SRDF to be nonempty and unchanged from startup, then
   add or replace collision object `object` at the pick location through an
   acknowledged `ApplyPlanningScene` request.
3. Reuse the validated startup model, build the stages below and capture the available
   joint-state cache for the final return goal.
4. Call `task_.init()` and `task_.plan(2)`. Require a successful result, a nonnull
   first solution with a start scene, and nonempty serialized task steps.
5. Verify that the configured descriptions did not change during planning.
   Save an immutable serialized solution, its full start scene and those
   descriptions; publish the preview/status and enter `Planned`.

Task replacement, assigning the startup model, stage construction, initialization, planning
and serialization run inside the callback's exception handlers. A handled
`InitStageException` or `std::exception` clears readiness and preview, enters
`Faulted`, and publishes planning failure. An older successful task cannot
become executable again after a failed replan.

The following stages are built only for a tool with the validated vacuum
capability. Planning records IO markers and does not require an available IO
service; service availability is checked during execution.

| Stage sequence | Behavior |
| --- | --- |
| Current state | Seeds planning from MoveIt's current scene/state. |
| Move to pick | OMPL `Connect` stage. |
| Approach object | Cartesian approach in TCP positive Z. |
| Grasp pose alternatives | Twelve orientations at 30-degree increments around object Z; each has `GeneratePose` and `ComputeIK`, with up to two IK solutions per pose. |
| Allow collision | Allow contact between `object` and the vacuum definition's validated contact links. |
| DO ON | Record pin 3, digital-output function 1, state 1.0. |
| Finalize approach | Require a full 0.1 m Cartesian contact approach. |
| Attach object | Attach `object` to the selected IK frame in the task's scene. |
| Lift object | Cartesian motion in TCP negative Z, 0.05–0.2 m. |
| Move to place | OMPL `Connect` stage. |
| Approach place and place pose IK | Full 0.1 m approach, `GeneratePlacePose` monitoring the attachment stage, and `ComputeIK` with up to ten solutions. |
| DO OFF | Record pin 3 state 0.0 and a three-second release delay. |
| Detach object | Detach the object in the task's scene. |
| Retreat | Cartesian motion in TCP negative Z, 0.1–0.3 m. |
| Forbid collision | Restore collision checking between `object` and the same validated contact links. |
| Return to start | OMPL `MoveTo` to the cached six-joint configuration, or named `home` if the cache is incomplete. |

Two Cartesian solver instances separate a 0.9 minimum path fraction from a
strict 1.0 fraction for contact moves; both use a 0.01 m step and 0.5 velocity
and acceleration scaling. Individual OMPL planner attempts and connect
stages have time limits. The code does not set an overall task deadline or
invoke MTC planning preemption.

The return goal is read from a separate `/joint_states` cache while building
the task. It is not extracted from the eventual first solution's initial
state, and freshness is not checked at that capture point. Execution separately
checks fresh received feedback against the prepared solution's actual start
state. Returning to these joints does not restore the physical tile to its
pick position or make the consumed plan executable again.

## Exact TCP A/B planning

The separate `tcp_poses_plan_target` callback validates the full two-pose request
and builds `CurrentState -> MoveTo(TCP A) -> MoveTo(TCP B)` with the startup
model's `tcp` IK frame. It adds no tile, yaw alternatives, approach/return motion,
attachment or vacuum stage, and never executes while planning. `none`, `kenos`
and `kenos_sl` can use this path. The read-only startup bool
`tcp_pose_planning_enabled` defaults to true and advertises this capability.

Both task modes reuse the mutex, PlanGate, immutable serialized solution, actual
start scene and description checks. Prepared TCP plans additionally store their
request ID. `execute_tcp_plan` requires the matching mode and ID under the same
lock before one-shot consumption and the existing scene/joint preflight.
`execute_task` remains tile-only. TCP execution disables the IO client and
rejects IO markers. No external execution ownership or cancellation is added.

TCP output topics are separate from tile outputs. Planning publishes progress
with `phase=planning`, then the correlated preview and `phase=ready` result;
terminal failure and cross-mode invalidation cannot revive a previous plan.
The request stamp is correlation only; MoveTo goal stamps are zero. See the
[wire contract](API.md#plan-and-review-exact-tcp-ab-targets) for exact fields,
topics and required client matching.

## Plan admission and execution checks

The [plan gate](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_gate.hpp)
has five states:

| State | Admission behavior |
| --- | --- |
| `Idle` | No executable plan; a new planning request is allowed. |
| `Planning` | Previous eligibility is gone; execution and overlapping planning are rejected. |
| `Planned` | One immutable prepared plan is eligible for an execution attempt. |
| `Executing` | Eligibility is already consumed; further execution/planning is rejected while work is active. |
| `Faulted` | No executable plan; a fresh planning request can establish new eligibility. |

An accepted scene update invalidates the plan and preview before applying the
box. Vacuum-task admission first requires the startup tool capability; rejection
invalidates any retained eligibility. For an eligible tool, `true` on
`/execute_task` requires `Planned` and a prepared plan. Admission
moves that plan into the callback and consumes readiness **before** constructing
execution clients or checking the start state. Success ends in `Idle`; a failed
attempt ends in `Faulted`. A preflight rejection also consumes the plan. There
is no retry of the old plan, even if its task data remains in memory.

The executor constructs `MoveGroupInterface` with a five-second action-server
wait, then the task callback performs its preflight before any task motion/IO:

1. Compare the node's current URDF/SRDF strings with the prepared descriptions.
2. Query the full relevant MoveIt scene and compare it with the selected
   solution's start scene using [plan validation](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/plan_validation.hpp).
   Joint positions must satisfy the configured tolerance. World geometry,
   octomap, attachments, collision permissions, fixed transforms, padding and
   scaling must match after normalization.
3. Require recent `/joint_states` receipt and compare those measured positions
   with the same planned start positions. Missing, duplicate or nonfinite
   joint data is rejected.

Scene query and box-application clients each allow two seconds for service
availability and three seconds for a response. False application acknowledgements,
missing responses, timeouts and exceptions prevent readiness or execution.
Removing a timed-out client request cannot cancel a server-side scene update.

Scene comparison is conservative: it normalizes timestamps and named-entry
ordering, and excludes noncollision metadata such as colors. Remaining geometry
is compared exactly; mathematically equivalent quaternion or shape representations
can require replanning. Multi-DOF robot state is unsupported for this fixed-base
arm. This is a point-in-time check, not exclusive scene ownership or continuous
validation: another writer can change the scene or command motion afterward.
Configured descriptions do not establish which physical tool is mounted.

## IO markers and execution

[`SetIOStage`](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/set_io_stage.hpp)
is a forward-propagating planning stage. It copies the incoming planning
scene and emits an empty `SubTrajectory` whose comment encodes an
`IO_MARKER`. It performs no IO calls during planning. The marker contains
service name, function, pin, state, service wait timeout, call timeout, and
post-switch delay.

Both vacuum tool definitions bind `/io_and_status_controller/set_io`, function `1`,
pin `3`, on/off states `1.0`/`0.0`, and a `3.0` second release delay. The same
definition supplies the marker fields and executor service, rather than separate
task-code literals. No IO service is contacted by startup tool validation.

[`CustomExecutor`](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/custom_executor.hpp)
walks the immutable solution message prepared when planning succeeded. A separate
overload accepts and serializes an MTC solution, but the task callback uses the
prepared message directly.
For recognized IO comments it parses function, pin, state, and delay, calls
`ur_msgs/srv/SetIO`, checks `response.success`, and waits for the delay.
The marker's service name and timeout values are not used: the client uses
the service passed to the executor constructor, with a two-second service
wait and a three-second response timeout. Ordinary trajectories are sent
through blocking `MoveGroupInterface::execute()` calls.

After a step succeeds, including its IO settle delay, the executor applies
`sub.scene_diff` before advancing. Empty trajectories and IO-only steps also
reach this completion boundary. The shared
[sequence helper](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/execution_sequence.hpp)
stops at the first failed operation or scene update. The task's allow-collision,
attachment, detachment and collision-restoration stages therefore update the
live MoveIt scene in execution order. Detachment preserves both the attached
body removal and serialized placed-world geometry.

The [scene-effect helper](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/scene_effects.hpp)
copies the planned message, clears its joint and multi-DOF joint states and sets
`robot_state.is_diff=true`. Attached objects, world geometry, collision permissions
and other scene fields remain intact. The scene's own `is_diff` flag is preserved:
MTC can serialize full snapshots as well as diffs. The cached solution is not
mutated. A full snapshot can replace world changes made after the preflight;
the preflight does not establish exclusive ownership of the scene.
This follows the scene-effect handling in the
[pinned MTC execution capability](https://github.com/moveit/moveit_task_constructor/blob/3cdf7f60fae2ca813de7b6616a654fef1bdb26cf/capabilities/src/execute_task_solution_capability.cpp),
while retaining the repository's custom GPIO handling.

All serialized effects use a relative `apply_planning_scene` service client in the
executor's reentrant callback group. The client waits up to two seconds for
availability and three seconds for the response. A successful acknowledgement
is required before continuing. A false/null response, unavailable service,
timeout or caught exception stops execution and is logged. Timeout removes the
pending client request but cannot cancel a server-side scene change; its outcome
is unknown. Scene application can also fail after partial processing. There is
no automatic retry, rollback, collision reset or vacuum release. Earlier
operations remain performed. The execution attempt consumes the prepared plan,
so inspection and fresh planning are required before another attempt. The
lifecycle gate does not automate recovery from partial physical effects.

## ROS and Unity-facing contract

Input topic names are relative to the node namespace; output names are
absolute. With the default namespace their effective names are:

| Topic | ROS type | Observable contract |
| --- | --- | --- |
| `/scene_setup_box` | `ur16e_interfaces/msg/Box` | Invalidate the prepared plan, then apply a box using fixed ID `object`; reports invalidation or failure on execution status. A pick request replaces it with fixed tile dimensions. |
| `/pick_place_poses_target` | `ur16e_interfaces/msg/PickPlacePoses` | Require vacuum capability, invalidate any old plan, then build and prepare a task; unsupported tools reject without a scene change. |
| `/execute_task` | `std_msgs/msg/Bool` | `true` requires vacuum capability and requests one validated execution attempt; `false` does nothing and is not cancellation. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Cache the message and steady-clock receipt time with sensor-data QoS; used for the return goal and execution preflight. |
| `/mtc/planned_trajectory` | `moveit_msgs/msg/RobotTrajectory` | Flatten the chosen solution's joint motion into a single preview path; publisher history depth 1, transient-local durability. |
| `/mtc/planning_status` | `std_msgs/msg/String` | JSON planning summary; publisher history depth 1, transient-local durability. |
| `/mtc/execution_status` | `std_msgs/msg/String` | Additive JSON lifecycle/result message with `status` and `reason`; depth 1, transient-local durability. |

The existing planning-status JSON shape is unchanged:

```json
{"overall_success":true,"stages":[{"name":"current","depth":1,"solved":1,"failed":0}]}
```

Stage counts describe the search's successful and failed stage solutions;
they are not execution progress. The trajectory flattener selects one joint
ordering, reorders segment positions, concatenates times, and omits empty
segments. It does not preserve velocities, accelerations, scene changes,
IO operations, or the three-second release delay. It publishes a joint-path
preview rather than a full task execution timeline.

The execution-status JSON has shape `{"status":"ready","reason":"..."}`.
Statuses are `ready`, `invalidated`, `planning_failed`, `rejected`, `executing`,
`succeeded` and `failed`; their meanings are listed in [the public API](API.md).
An empty preview replaces the old trajectory on invalidation, planning failure,
unsupported-tool rejection and execution admission. Unsupported-tool rejection
also publishes `overall_success=false` and execution status such as
`{"status":"rejected","reason":"Mounted tool 'none' does not support vacuum pick/place"}`.
These legacy tile outputs and commands carry no request/plan ID. Separate TCP outputs and `execute_tcp_plan` use the request token described above.
Execution status is not per-stage progress or a cancellation interface.
The source comments name a Unity `UR16eTrajectoryVisualizer`; its C#
implementation is not present in this repository. End-to-end Unity parsing,
coordinate conversion, and late-subscriber QoS behavior are not established
by these publishers alone.

## Callback and lifecycle behavior

Scene setup, planning and execution use a reentrant command callback group.
Each attempts a nonblocking lock on the same task mutex and holds the lock
through the complete callback, including service waits. A callback that overlaps
active work reports `rejected` without touching the task/gate/prepared plan.
Joint feedback has its own callback group and mutex; scene and IO service
responses use reentrant groups so they can complete during command waits.

Busy rejection applies when callbacks overlap, not when messages were sent.
DDS/executor-delayed callbacks can start after earlier work finishes. A consumed
plan still cannot execute twice, but an old `Bool` trigger delivered after a new
plan is prepared cannot be associated with its original request. The tile Bool path has no
plan ID; neither path adds cancellation, shared ownership with the basic commander, or verification
of the physical tool. Basic TCP/frame semantics are described in [the API](API.md);
the tool capability gate controls this node's vacuum tasks only.

## TCP contact targets and execution-frame correction (2026-10-22)

The additive `pick_place_tcp_target` callback enters the same vacuum planning
gate with a different, validated input contract: desired TCP contact positions
and outward +Z normals. It derives the planning object's centre along the tool
normal rather than reference-frame vertical. Grasp and place each sample twelve
local-Z twists; place uses monitored `GeneratePose`/`ComputeIK` alternatives
directly for TCP, leaving the attached object to follow its rigid grasp transform.
The original surface callback and its historical object placement remain intact.
See [the contact wire contract](API.md#tool-contact-pickplace-targets).

Execution validation now uses MoveIt's actual transform serialization: source
is `header.frame_id`, while multiple entries legitimately share planning-target
`child_frame_id`. Empty names and duplicate sources still reject execution.
After structural validation, only transform sources referenced by actual world
objects, attachments or nonempty octomap geometry participate in exact scene
comparison. Unused measured-controller TCP transforms can vary with encoder
feedback without invalidating unchanged collision geometry. Referenced frame,
geometry, permissions, padding, scaling and joint checks remain protected.

Before a TCP-only plan, the node can remove its own unchanged, unexecuted
synthetic task-preview object. It records the acknowledged object's serialized
world geometry after creating it; changed external geometry rejects cleanup.
Manual scene objects, attached objects, objects from a previous node session and
objects after any vacuum execution attempt are retained. No shared scene lock
or external ownership protocol is added; external writers can still race a
point-in-time check.

## Refresh

- Last refreshed: 2026-10-22 for exact TCP A/B planning and Safe&Light selection/arguments; 2026-09-14 for startup tool/model validation and vacuum
  eligibility, preserving immutable plan admission, preflight and ordered scene effects.
- Source basis: `ur16e_mtc_gpio_stage` implementation, launch, scripts and build
  metadata; `ur16e_interfaces/msg`; MoveIt configuration; upstream MTC
  execution source linked above; `plan_gate.hpp`, `plan_validation.hpp`,
  [tool definitions](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/tool_definition.hpp),
  [tool/model validation](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/tool_model_validation.hpp)
  and [startup parameters](../../ros_ws/src/ur16e_mtc_gpio_stage/include/ur16e_mtc_gpio_stage/startup_parameter.hpp).
  Baseline commit `28fda0c03f5952fc45bf4ad65a5d7831354fc80d` plus the valid-plan and tool-eligibility changes.
- Limitations: see [test coverage and results](BUILD_AND_TESTING.md). The exact TCP package compiled and its focused tests ran offline; no MTC planning run, application launch, mock execution or hardware operation was performed. Historical reasons for implementation choices are not inferred
  as decisions here.
