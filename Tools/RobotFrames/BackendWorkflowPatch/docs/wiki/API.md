# Public interfaces

The main launch exposes ROS-TCP-Endpoint at `0.0.0.0:10000`. Custom messages live in [`ur16e_interfaces/msg`](../../ros_ws/src/ur16e_interfaces/msg). There are no application `.action` or `.srv` definitions in the checkout.

## Application inputs

| Topic | Type | Owner | Behavior |
| --- | --- | --- | --- |
| `/tcp_pose_target` | `geometry_msgs/PoseStamped` | Basic commander | Plans and executes a `tcp` goal using the message's frame and timestamp. |
| `/tcp_pose_array_target` | `geometry_msgs/PoseArray` | Basic commander | Validates the entire array and resolves its shared frame/time before planning and executing each `tcp` goal in order. This is not a blended Cartesian path. |
| `/pose_target` | `geometry_msgs/PoseStamped` | Basic commander, legacy opt-in | Disabled by default. Interprets the pose as a wrist goal in `mir_base_footprint`, ignores its header and converts the goal to `tcp`. |
| `/pose_array_target` | `geometry_msgs/PoseArray` | Basic commander, legacy opt-in | Disabled by default. Interprets poses as wrist goals in `ur_base_link`, ignores the header and converts goals to `tcp`. |
| `/scene_setup_box` | `ur16e_interfaces/Box` | MTC node | Invalidates the prepared plan/preview, then applies the world collision object named `object` with acknowledgement. |
| `/pick_place_poses_target` | `ur16e_interfaces/PickPlacePoses` | MTC node | Requires vacuum capability, then invalidates the previous plan/preview, adds the tile, and builds a plan. `tool=none` rejects before a scene change. |
| `/pick_place_tcp_target` | `ur16e_interfaces/PickPlacePoses` | MTC node | Plan-only desired pick/place TCP contacts; position and outward tool +Z are fixed, twist is sampled at both ends. Requires vacuum capability. |
| `/execute_task` | `std_msgs/Bool` | MTC node | Tile-only: `true` requires vacuum capability and one prepared tile plan; rejects TCP plans. `false` does nothing and does not cancel. |
| `/tcp_poses_plan_target` | `geometry_msgs/PoseArray` | MTC node | Plan-only exact TCP A then B with a nonzero request token; returns a correlated preview without motion or IO. |
| `/execute_tcp_plan` | `std_msgs/String` | MTC node | Execute the exact reviewed TCP request ID once, after frozen-plan preflight; wrong mode/ID is rejected without consuming another plan. |
| `/joint_states` | `sensor_msgs/JointState` | UR stack; consumed by MTC | Cached for the return goal and start-position validation, with a steady-clock receipt time for freshness checking. |

The input topic names and `commander/motion_status` are relative and resolve as above in the default namespace; MTC output names are absolute. Basic commander callbacks serialize its commands, so later commands can wait in the subscription queue. MTC command callbacks instead share a nonblocking task lock: callbacks overlapping planning, execution or a scene update are rejected. DDS/executor delays can deliver a callback after the active work has finished; this is not a message-arrival-time busy filter. The two commanders do not share execution ownership.

## Basic TCP commands and legacy migration

The [basic commander](../../ros_ws/src/ur16e_commander/src/ur16e_commander.cpp) explicitly selects `tcp` for every pose target. At startup it requires `tcp` and `ur_wrist_3_link` in the robot model, verifies their connecting chain is fixed, and caches the model's wrist-to-TCP transform. The `none`, `kenos` and `kenos_sl` variants provide this fixed chain; it does not verify the physically mounted tool.

The strict topics require nonempty frame names without a leading slash, whitespace or control characters; timestamps with nonnegative seconds and nanoseconds below one billion; and finite positions/quaternion components. Quaternion norm must be within `0.001` of one, after which it is normalized. Empty arrays and malformed poses are rejected before any motion, including when only a later array entry is invalid. [Target preparation](../../ros_ws/src/ur16e_commander/include/ur16e_commander/pose_targets.hpp) resolves one source-to-planning-frame snapshot for the whole command and freezes every target in the startup planning frame. A zero timestamp requests the latest available transform during admission; a nonzero timestamp requests the transform at that ROS time. Unavailable history or disconnected frames reject the input, with no fallback to the latest transform or another frame. A frame already equal to the planning frame needs no TF lookup. Prepared targets have zero timestamps because their coordinates are already frozen in the planning frame.

The read-only startup parameter `transform_timeout` defaults to **1.0 s**, must be finite and greater than zero, and must not exceed **10.0 s**. The commander uses a steady-clock deadline to bound waiting for TF. This timeout does not bound planning or execution. Once targets are resolved, later source-frame movement does not move the goals. Each move is planned from the current robot state after the preceding execution succeeds. The [sequence helper](../../ros_ws/src/ur16e_commander/include/ur16e_commander/command_sequence.hpp) stops at the first planning or execution failure/exception. Earlier moves are not rolled back; the failed execution may have moved partially.

The read-only startup parameter `enable_legacy_pose_topics` defaults to `false`; [top-level bringup](../../ros_ws/src/ur16e_bringup/launch/ur16e_start.launch.py) exposes the same launch argument. When enabled, the deprecated topics retain their historical wrist-coordinate conventions: single poses use `mir_base_footprint`, arrays use `ur_base_link`, and both use latest TF while ignoring supplied headers. The adapter applies the model's fixed wrist-to-TCP offset before entering the same TCP pipeline. When disabled, those subscriptions do not exist.

Unity migration requires sending the desired tool working-point pose on the new topic, with the correct ROS frame and timestamp, and removing any client-side wrist compensation. Renaming a legacy topic alone does not convert wrist coordinates to TCP coordinates. Unity sources are absent from this checkout, so client conversion has not been verified. The [single-pose](../../ros_ws/src/ur16e_commander/src/test_send_pose.sh) and [array](../../ros_ws/src/ur16e_commander/src/test_send_poseArray.sh) shell publishers send explicit TCP goals on the new topics, using `mir_base_footprint` and `ur_base_link` respectively. Both publish once with zero timestamps for latest-TF admission and need no legacy opt-in. Their example numbers now describe TCP poses and do not preserve the previous wrist configurations. The separate [test_moveit](../../ros_ws/src/ur16e_commander/src/test_moveit.cpp) executable remains a wrist-target demonstration.

## Plan and review exact TCP A/B targets

The MTC node exposes the read-only startup bool `tcp_pose_planning_enabled`
(default `true`). A client can read it through
`/mtc_test_task_node/get_parameters` before publishing a plan request. This is
separate from the basic commander's TCP topics, which plan **and execute**.

`/tcp_poses_plan_target` accepts exactly two finite poses, A then B, with
`header.frame_id=ur_base_link`. Quaternions must be finite and nonzero; they are
normalized without changing orientation. The shared nonzero valid stamp is an
opaque request token, with ID `sec:nanosec` in decimal without padding. Generate
a fresh token for each request. Actual MoveTo goal stamps are zero, so this token
is never interpreted as historical TF time. The complete request is validated
before task stages are built.

Planning builds `CurrentState -> MoveTo(TCP A) -> MoveTo(TCP B)` with OMPL and
the startup model's validated `tcp` frame. Full target orientations are retained;
there is no tile, grasp-yaw sampling, tool-length compensation, IO or automatic
return. Paths between targets are collision-aware joint paths, not prescribed
straight Cartesian lines. All three startup tools, including `none`, support
this mode. Planning never executes the solution.

TCP planning status has the existing `overall_success` and `stages` fields plus
`message`, `request_id` and `phase`. Initial `phase=planning` with false success
means progress. Final success uses `phase=ready`; terminal errors use `failed`,
and cross-mode replacement uses `invalidated`. The selected preview echoes the
token in `joint_trajectory.header.stamp`. A client must require both a matching
ready result and nonempty matching preview before offering execution; the two
topics may arrive in either order. Empty invalidation previews are not completed
plans, and late results for earlier tokens must be ignored.

`/execute_tcp_plan` receives the reviewed ID as `std_msgs/String`. Under the task
mutex, it requires a prepared TCP plan with that exact ID before consuming its
single execution attempt. The same frozen-solution, description, start-joint,
fresh-feedback and collision-scene preflight applies. Its executor creates no IO
client and rejects any IO marker. Correlated `/mtc/tcp_execution_status` contains
`status`, `reason` and the **incoming execute ID**, including for busy/mismatch
rejection; statuses are `rejected`, `executing`, `succeeded` and `failed`.

Tile and TCP requests share one prepared-plan slot; a new admitted request
replaces the old plan. `/execute_task` remains tile-only and rejects a TCP plan.
Request matching does not establish shared robot ownership, cancellation or
protection against an external motion/scene writer.

## Application outputs

| Topic | Type | Semantics |
| --- | --- | --- |
| `/commander/motion_status` | `std_msgs/String` | JSON with `status`, local input `topic`, `move_index`, `move_count` and `reason` for basic pose commands. |
| `/mtc/planned_trajectory` | `moveit_msgs/RobotTrajectory` | Best solution flattened to positions and cumulative times for a hologram preview. IO, release pauses, scene diffs, velocities and accelerations are omitted. |
| `/mtc/planning_status` | `std_msgs/String` | JSON containing `overall_success` and per-stage `name`, `depth`, `solved`, `failed`. It reports planning, not execution progress. |
| `/mtc/execution_status` | `std_msgs/String` | Existing MTC lifecycle JSON containing `status` and `reason`. |
| `/mtc/tcp_planned_trajectory` | `moveit_msgs/RobotTrajectory` | Exact A/B preview; joint-trajectory header stamp echoes its request token. |
| `/mtc/tcp_planning_status` | `std_msgs/String` | Correlated JSON with `overall_success`, `stages`, `message`, `request_id` and `phase`. |
| `/mtc/tcp_execution_status` | `std_msgs/String` | Correlated TCP execution JSON with `status`, `reason` and incoming execute `request_id`. |

All seven listed outputs use depth-one transient-local publishers. Actual late-joining delivery through ROS-TCP-Endpoint depends on the subscriber QoS and has not been tested here. The basic commander and legacy tile interfaces have no request/plan identifier; the separate exact TCP interfaces carry the request token described above. An empty trajectory replaces the previous MTC preview on invalidation, planning failure and execution admission. The planning-status JSON shape is unchanged; an earlier `overall_success=true` is not current execution authorization.

Basic commander status has this shape:

```json
{"status":"executing","topic":"tcp_pose_array_target","move_index":2,"move_count":3,"reason":""}
```

`planning` and `executing` identify the current move; `succeeded` reports completion of the whole command; `failed` identifies the first failed move. `move_index` is one-based, except `rejected` uses zero for whole-input validation/transform failure before motion. `topic` is the local subscription name, without a namespace or leading slash. This status has no command ID or cancellation facility and cannot reliably correlate multiple queued commands of the same type.

MTC execution-status messages have this shape:

```json
{"status":"ready","reason":"A successful plan is available for one execution attempt"}
```

| `status` | Meaning |
| --- | --- |
| `ready` | Planning produced an immutable prepared solution eligible for one attempt. |
| `invalidated` | New planning replaced the old plan, or a scene update completed and requires replanning. |
| `planning_failed` | Task construction, initialization, planning or preparation failed; no plan is ready. |
| `rejected` | Unsupported tool, busy callback, no eligible plan, or failed execution preflight. Unsupported-tool rejection clears readiness/preview; preflight rejection consumes the plan; busy rejection does not consume another callback's plan. |
| `executing` | Preflight passed and the single execution attempt is starting. |
| `succeeded` | Execution completed; another attempt needs a new plan. |
| `failed` | Scene update or execution attempt failed. Inspect state before planning again; completed physical effects are not rolled back. |

This is a lifecycle/result topic, not a per-stage progress stream or cancellation API. A delayed old `Bool` trigger cannot be associated with its intended plan and may reach a newer eligible plan.

## Execution admission and configuration

The [MTC task node](../../ros_ws/src/ur16e_mtc_gpio_stage/src/test_task.cpp) requires `Planned` state and a prepared solution before using it. It consumes readiness before client setup/preflight and never restores the same plan after success, failure or preflight rejection. Construction, initialization and planning exceptions invalidate readiness and preview.

The [MTC launch](../../ros_ws/src/ur16e_mtc_gpio_stage/launch/mtc_commander.launch.py) exposes `execution_start_tolerance` (default **0.01 rad**) and `execution_joint_state_max_age` (default **1.0 s since feedback receipt**, not message timestamp). Both must be finite and positive and are read once at startup. Nonempty URDF/SRDF parameters are also required; changing them after startup requires restarting the task node with consistent configuration.

The same launch `tool` value generates URDF/SRDF and becomes the node's read-only startup `tool` parameter (`none` by default). Read-only `ik_frame` defaults to `tcp` and must match that tool definition. The node loads its model once and validates tool/model consistency before creating command subscriptions; unknown tools, missing required links/collision geometry, invalid fixed chains/groups or a different IK frame stop startup. New tasks reuse the validated model. See [robot/tool configuration](ROBOT_AND_TOOLS.md).

With `tool=none`, scene setup, basic TCP motion and exact TCP A/B plan/review/execute remain available. Tile pick/place and legacy `true` execution requests reject before a scene change, task-stage construction, executor creation or IO; they invalidate the plan, clear its preview and publish `overall_success=false` alongside `{"status":"rejected","reason":"Mounted tool 'none' does not support vacuum pick/place"}`. `false` execution requests remain no-ops. Kenos task eligibility does not require the IO service to be present at startup/planning; planning emits markers, and execution checks service availability. Configured capability is not proof of physical tool identity.

After a five-second MoveGroupInterface action-server wait, preflight checks descriptions, the solution's start joints, the full relevant scene and recent joint feedback. Scene query/application calls have a two-second availability wait and three-second response timeout. Collision geometry comparisons normalize named-entry ordering and timestamps, then compare remaining geometry exactly. They can reject equivalent alternate representations. Checks do not verify the mounted physical tool, lock external scene writers, or prevent the basic commander from issuing motion after validation. See [task execution](TASK_EXECUTION.md) for state transitions and the failure boundary.

## Message and frame conventions

`PickPlacePoses` contains two stamped poses, without object dimensions, object ID, tool ID, request ID or an explicit top-surface/center convention. The implementation treats the positions as tile top-surface centers and subtracts half the thickness along the reference frame's Z axis. This is valid only for the orientation assumption noted in the source; a generally rotated tile needs the offset along its own normal.

`Box` contains `boxx`, `boxy`, `boxz` and `position: PoseStamped`. The consumer always chooses `object` as its ID and does not validate positive finite dimensions or the pose before applying it. The separate environment node inserts `ground` with default dimensions `[5,5,0.02]` and top Z `-0.04` in its resolved planning frame.

The basic command endpoint is explicitly `tcp`; the MTC launch defaults `ik_frame` to `tcp`. The model's TCP depends on the selected `none`, `kenos` or `kenos_sl` tool, as described in [robot and tools](ROBOT_AND_TOOLS.md). Only the opt-in legacy adapter substitutes historical frames and wrist semantics.

The standalone `Rpy`, `PoseRpy` and `PoseArrayRpy` types are generated, but the main command subscriptions use standard quaternion messages. The separate `test_moveit` demonstration includes commented joint and named targets; these are not public commander subscriptions.

## Tool-contact pick/place targets

`/pick_place_tcp_target` is an additive replacement for XR clients that place
tool references. The read-only startup bool `pick_place_tcp_planning_enabled`
defaults to `true`; clients must check it before publishing. Both fields of
`PickPlacePoses` must name `ur_base_link`, contain finite positions and nonzero
finite quaternions. The backend normalizes rotations and treats the contacts as
static base-relative poses, clearing their timestamps.

Each position is the desired tool TCP contact point, including any deliberate
client depth adjustment. Tool +Z points outward through the suction foam toward
the object. Pick and place keep that normal while independently sampling twelve
tool twists at 30-degree intervals. The returned trajectory shows the chosen
clocking. These samples are finite alternatives, not continuous yaw optimization.

The planning tile remains 100 x 100 x 4 mm. Its centre is contact position plus
half its thickness along tool +Z; the offset rotates with the supplied tool
orientation. The pick pregrasp is 100 mm behind the contact along tool -Z, and
the contact approach advances along +Z. Place candidates directly address TCP,
so an attached object's arbitrary grasp twist does not change the requested
place point/normal. Client +5/-5 mm depth steps shift the desired TCP itself;
the backend adds no unrequested compression, force control or compliance.

This endpoint uses the existing `/mtc/planning_status`,
`/mtc/planned_trajectory`, `/mtc/execution_status` and separate
`/execute_task=true` vacuum execution flow. These legacy outputs have no request
ID. `/pick_place_poses_target` retains its original surface/object convention;
renaming that topic without changing pose meaning is incorrect.

## Refresh

- Last refreshed: 2026-10-22 for exact TCP A/B planning and Safe&Light TCP model selection; 2026-10-21 for the TCP shell examples; other interface descriptions retain their 2026-09-14 source review.
- Source basis: [basic commander](../../ros_ws/src/ur16e_commander/src/ur16e_commander.cpp), [target preparation](../../ros_ws/src/ur16e_commander/include/ur16e_commander/pose_targets.hpp), [move sequencing](../../ros_ws/src/ur16e_commander/include/ur16e_commander/command_sequence.hpp), [MTC node](../../ros_ws/src/ur16e_mtc_gpio_stage/src/test_task.cpp), [MTC launch](../../ros_ws/src/ur16e_mtc_gpio_stage/launch/mtc_commander.launch.py), [environment node](../../ros_ws/src/ur16e_environment/src/environment_updater.cpp), [messages](../../ros_ws/src/ur16e_interfaces/msg), [bringup](../../ros_ws/src/ur16e_bringup/launch/ur16e_start.launch.py).
- Limitations: contracts are source-derived and the exact TCP package compiled offline. ROS-TCP transport, planning reachability, bridge QoS and hardware behavior remain unverified. Companion Unity client sources are maintained outside this checkout.
