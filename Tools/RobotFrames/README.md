# Offline robot-frame comparison

## Four-point controller placement

The interaction scene starts with **Place robot with 4 points**. Keep the real
robot still in a bent pose, with `/joint_states` streaming. Touch the centre of
each indicated blue rear cover with the controller contact tip and press trigger:
**Shoulder → Elbow → Wrist 2 → Wrist 3**. These are the four covers in the supplied
photo. The XR robot appears after the fourth accepted touch. Undo, Restart,
Cancel, and Use QR instead are available in the placement menu.

`RobotPointRegistration` fits a rigid rotation and translation using all four
ordered correspondences and the calibrated model at the first touch's live joint
pose. It estimates no scale. Incomplete/stale feedback, robot movement over
0.5 degrees, changed models, repeated points and nearly collinear geometry are
rejected. A failed fit remains in the picker. Default acceptance limits are
10 mm RMS and 20 mm worst-point residual; both values appear after placement.
Residuals measure consistency of the four touches, not absolute physical accuracy.

**Verify the physical contact tip first.** The 6 mm marker shows the point being
sampled. OpenXR's pointer origin is the default; it is not a measured physical
tip. Set **Controller Tip Offset In Pointer** on the existing `URInstantiator`
in the scene to the measured local Unity XYZ offset in metres. An explicitly
added `RobotPointRegistration` component exposes its own tip offset, cover
definitions and fit thresholds. A common tip error can produce a low residual
and a shifted robot. The cover defaults come from the actual visual meshes;
see [landmark derivation](controller_point_landmarks.md) for coordinates and
source-mesh verification. Check the resulting overlay on the physical robot.

Point placement requires no QR-to-base distance and does not overwrite saved
QR alignment profiles. Planning/capture are blocked while picking; placement
never sends robot motion. Existing placement is hidden during a retry and
restored on Cancel when its tracking session is still valid. Native tracking
origin resets and app suspension invalidate the fit and require four new touches.
Moving the XR Origin transform retains the fit in tracking space. Contacts and
world placement are session-local and are not restored after restarting the app.

`prepare_point_registration_validation.py` prepares an isolated Unity project
under `Library/PointRegistrationValidation`, using the actual placement,
subscriber, input, menu and rig code. QR hardware is inert and obtaining a ROS
connection throws. Run its EditMode fixtures `RobotPointRegistrationMathTests`,
`RobotCoverLandmarkTests`, `RobotPointRegistrationWorkflowTests` and
`PointPlacementMenuTests`; the menu fixture can also render an offline preview.

## Choosing TCP or MTC pick/place in XR

The shared robot menu in `UR16eInteraction` has two separate task buttons:
**TCP A → B** and **MTC pick / place**. The selected button is highlighted.
Hold the controller bumper to capture A or the pick tool pose; double press it to
capture B or the place contact point. Only the selected task's captured references and
rotation handles are visible. Switching retains each task's targets but clears
both planning states and the ghost trajectory; a new Plan is required before
Execute. Mode selection is blocked while aligning the robot or turning a target.

Captured references have short **A**, **B**, **Pick** or **Place** labels beside
them. The labels face the viewer, follow their reference after rotation/depth
edits, and hide with the inactive mode. Their text is about 2 cm high; coordinates
and instructions remain in the menu and diagnostic logs.
`target_name_validation_summary.json` records compilation of 81 runtime sources,
29/29 existing capture/mode checks passing, and the inspected
[`label preview`](TargetNameValidation/named-targets.png). No headset app build
or hardware operation is part of those checks.

TCP targets specify the complete tool pose, including rotation about tool +Z.
MTC targets specify desired TCP contact points and tool +Z normals. ROS samples
twelve tool twists at 30-degree intervals independently at pick and place; it
also generates approach, lift, placement, retreat and vacuum stages. The contact
normal is constrained, while rotation around it is free. Review the actual
solved tool rotation in the trajectory.

TCP A/B shows the selected tool geometry. MTC pick/place shows a small ball at
each contact point with a single arrow along tool +Z, and the same ball/arrow
as the live aim preview. It shows no fixed tool clocking or curved twist rings.
Trigger-drag the ball to turn the normal. Hover a placed reference and press
the bumper for its contextual menu. MTC targets have **+Z 5 mm**,
**-Z 5 mm** and **Reset depth** controls for contact compression/backoff. Depth
steps follow the current tool normal and invalidate any reviewed plan. Close the
menu before rotating, planning or executing. TCP targets retain exact captured
positions; their context menu explains rotation controls.

The ball/arrow materials are bundled under `Resources/RobotTargetMarkers` so
Android includes their opaque and transparent URP Unlit shader variants.
Runtime-only `Shader.Find` references were absent from the previous full Android
build; scripts-only builds kept reusing that player data. **Make a full Android
Build/Build And Run after this fix; scripts-only rebuilding is insufficient.**
Recapturing also restores incomplete or hidden marker geometry while retaining
the short labels and exact contact pose.

`marker_rendering_validation_summary.json` records 80/80 focused native tests,
82 runtime sources compiling without errors, and an actual-controller render
using the mobile URP settings. Each Pick/Place ball, shaft and arrowhead must
render colored pixels independently of its label; see the
[`controller preview`](MarkerRenderingValidation/controller-preview.png).
`marker_player_validation_summary.json` additionally records a full isolated
Android player-data export retaining both materials and the Unlit shader, with
opaque, transparent and instanced Vulkan variants compiled. This export does
not install the app or test headset stereo rendering.

MTC requires a matching Kenos or Kenos SL model and the new capability-gated
`/pick_place_tcp_target` interface. It constructs the virtual object centre half
its thickness beyond the contact along tool +Z, including for tilted contacts.
The original surface-pose API remains compatible for older clients; this Unity
workflow does not use it. Neither endpoint adds a hidden compression distance.

Both tasks use **Plan preview** followed by a separate **Execute plan**.
MTC keeps listening after Execute consumes the reviewed permission: ROS
acknowledgement, progress, success or rejection now appears in the main menu.
No acknowledgement after 10 seconds reports an unknown outcome and keeps the
request consumed; Unity never retries automatically. A late outcome is still
accepted. New Plan/Execute is blocked until the execution attempt resolves.
Execution tracking survives target edits, tool/alignment changes, mode switches
and component disable. Those changes revoke the reviewed preview; they cannot
cancel a command already sent. Both workflows remain blocked until its outcome
arrives, and the main menu keeps showing MTC execution feedback across modes.
MTC execution now additionally requires its successful result and the actual
displayed trajectory, and consumes local readiness once. Local rejections and
planning timeouts are shown in the menu. The legacy MTC messages have no request
ID: callbacks are ignored outside a deliberate current MTC request, but an old
response arriving after a new MTC request cannot be distinguished by this
legacy protocol. Exact TCP requests use their separate correlated interface.

`mtc_execution_validation_summary.json` records 205/205 native checks passing,
including 20 execution lifecycle and actual public-action cases with a
non-network publish substitute. `mtc_execute_scene_diagnosis_20261023.json`
confirms the reported silent Execute reached ROS, which rejected three harmless
world-object quaternion rounding differences (at most 6.94e-18). The Unity
feedback fix does not bypass that ROS scene check. The approved backend
[precision fix](ExecutePrecisionProposal/README.md) is applied and built offline;
all 14 plan-validation cases pass. Nine of ten package test targets pass, with
the existing scene-effects assertion reproduced on the unchanged baseline.
The complete recorded planned/current scenes also pass the compiled validator;
the unchanged validator rejects the same data with the original error.
The precision fix was deployed to Nordbo on **2026-10-28** with an MTC-only
restart. The robot driver, MoveIt and ROS–Unity endpoint retained their original
processes. Plan again before Execute; this backend correction needs no additional
Unity rebuild.

The initial read-only **2026-10-28** Nordbo audit confirmed the same rejection:
the source, installed header and running MTC executable all predated the fix.
The fresh world's only difference is one quaternion coefficient by
`6.94e-18`; the octomap is unchanged. The complete fresh scenes pass the fixed
compiled validator and fail the old one. See
`mtc_execute_diagnosis_20261028.json` and `ExecutePrecisionValidation_20261028/`.
The durable Nordbo checkout and running container now carry the approved fix.
The verified `urmir-jazzy:latest` and `urmir-jazzy:mtc-precision-20261028` images
also contain it; the previous image is retained for rollback.
Deployment records, exact parameter/model checks and the rollback paths are in
`ExecutePrecisionValidation_20261028/`. No robot motion or IO was requested.

MTC's initial reset can contain an unsolved `task pipeline` root stage. Unity
treats that as initialization and continues waiting for the final result.
Failed IK alternatives inside an overall successful result do not make the
task fail. Success still requires its trajectory to be displayed before Execute;
waiting for the robot preview to initialize is reported separately from a ROS
planning failure. `mtc_success_live_20261023.json` is the read-only Nordbo success
snapshot used for the offline 97-point trajectory replay.
`mtc_menu_validation_summary.json` records 167/167 native tests passing,
including all 90 delivery orders across the three MTC topics and both orders
for the actual successful Nordbo result/trajectory. It also covers native XRI
menu grab/release and interruption recovery. Runtime compilation covers 82
sources without errors; the inspected menu render is
[`MtcMenuValidation/mtc-menu.png`](MtcMenuValidation/mtc-menu.png). These checks
send no planning, execution or IO requests and do not install the headset app.

## Animated arm and tool preview

Both planning modes display a separate cyan arm and selected tool using the
same calibrated ROS link transforms as the live replica. New plans prepare
their first pose with the animated arm hidden to avoid covering the live robot.
**Play** or **Replay** reveals it; Pause and subsequent timeline scrubbing keep
it visible for inspection. Scrubbing before the first Play keeps it hidden.
The path and static waypoint previews remain available for plan review.
In the main menu, **Play/Pause** controls virtual
playback, **Replay** restarts it, and the timeline inspects any point while
pausing playback. Speed choices are **0.25x**, **0.5x**, **1x** and **2x**.

Playback follows ROS waypoint timestamps and interpolates joint positions
between samples, preserving the supplied joint winding. The live robot replica
continues to show joint feedback. Playback controls send no ROS commands and
do not consume the reviewed plan; real movement still requires **Execute**.
Changing targets, tool, task mode or calibration clears the old preview.
The flattened MTC motion preview does not simulate suction, object attachment
or controller IO pauses.

`arm_playback_validation_summary.json` records **185/185 passing native checks**,
including playback, both planning workflows and XR menu interaction. The actual
97-point MTC record (8.569 s) is checked at every waypoint against the Kenos SL
TCP. Runtime compilation covers 82 sources without errors. Three inspected
arm/tool renders and the updated menu are in `ArmPlaybackValidation/`.
These checks use no ROS connection, robot command or headset deployment.

`preview_visibility_validation_summary.json` records the later **207/207 native
checks passing**, including 20 playback cases and both planning workflows.
It checks the actual animated robot's renderers stay inactive until Play/Replay,
remain visible when paused, and hide again for a new plan. Runtime compilation
covers the current 82 sources without errors.

## Exact TCP targets and planning

`UR16eInteraction` captures exact TCP poses by default. Hold the controller bumper
to capture A; double press it to capture B. The live tool copy shows the exact
pose before capture. Grab a target's tool or curved arrows to change its full
orientation, then use **Plan preview** in the shared robot menu. The backend
plans from the current robot state through A to B; it adds no tile, grasp yaw,
contact offset, suction action or return motion.

**Execute plan** becomes available only after the matching successful result and
its actual ghost trajectory are displayed. It sends the reviewed request ID for
one execution attempt. Capturing/turning targets, changing tool models, losing
the connection, rescanning QR or replacing calibration invalidates readiness.
Rescan blocks coordinate-dependent actions immediately; after a stable scan,
press the trigger to realign the existing robot and then plan again.

The applied ROS source update and wire contract are documented in
[`Proposals/README.md`](Proposals/README.md). It must be deployed separately to
the backend used by the headset. Unity checks the read-only
`tcp_pose_planning_enabled` capability and reports an unavailable backend in the
menu. The legacy direct-TCP topics execute immediately and are never used by
the Plan preview action.

The earlier `menu_follow_validation_summary.json` covers the retired automatic
follow behavior. The menu now stays where it is placed and faces the viewer;
use its grab bar to move it with the controller laser.
`task_mode_validation_summary.json` records 47 passing
task-selection tests, including 18 new mode/MTC cases using the actual controllers and
robot trajectory clones. Both callback arrival orders, initial ROS reset status,
late messages, mode/target invalidation, timeout and single-use review are checked.
`runtime_compile_summary.json` records the current runtime compilation.

`normal_target_validation_summary.json` records **85/85 passing native Unity
tests** after replacing MTC tool copies with contact balls and normal arrows.
The displayed arrow is checked against the exported contact normal, with the
depth/menu, input interruption, mode, TCP and menu-follow regressions passing.
Runtime compilation covers 80 sources with zero errors. The inspected render is
[`NormalTargetValidation/normal-contact-context.png`](NormalTargetValidation/normal-contact-context.png).
These checks do not build/install the headset app or access ROS/hardware.

The earlier `tool_contact_validation_summary.json` records **93/93 passing native Unity
tests** for the tool ghosts, context depth controls, input interruption recovery,
mode switching, TCP planning and menu following, with matching source hashes.
The inspected runtime render is
[`ToolContactValidation/tool-contact-context.png`](ToolContactValidation/tool-contact-context.png).
`new_workflow_backend_validation_summary.json` records the ROS build, all nine
fixed-frame guard cases and four contact cases passing. Eight of nine package
CTest targets passed; the unchanged scene-effects ACM assertion remains a known
baseline failure. Offline integration generated a 74-point contact trajectory,
then an 11-point TCP trajectory while preserving ground geometry and removing
only the owned, unexecuted preview object. These checks used a network-isolated
container without drivers or any Execute request. Neither the Nordbo backend
nor the headset app has been deployed by this change.

Earlier broader records are `alignment_workflow_validation_summary.json` (192
workflow/model/menu tests), `alignment_validation_summary.json` (36 actual-SDK
QR, alignment and filesystem tests), and `Proposals/TcpPlanningVerification.json`
(the applied ROS package build/tests). The two alignment-era Unity suites covered
217 unique cases, with 11 profile cases run in both. `tcp_validation_summary.json`,
`qr_validation_summary.json` and `unity_validation_summary.json` are older snapshots.
No headset tracking or robot execution is exercised by these offline checks.

## Aligning and saving the virtual robot in XR

In `UR16eInteraction`, scan the robot's QR and press the trigger to place the
virtual robot. Choose **Align robot** in the robot menu. The same panel switches
to position and orientation controls; the panel stays in place while the robot
overlay moves.

- Select **100 mm**, **10 mm** or **1 mm**, then use Left/Right, Towards/Away and
  Up/Down to align the base. Horizontal moves follow the view direction at the
  start of alignment; turning your head later does not change the button axes.
- Select **5 degrees**, **1 degree** or **0.1 degrees**, then adjust yaw, pitch and
  roll. Rotations keep the base position fixed. Match the base and multiple arm
  links.
- **Save & finish** keeps the alignment and returns to the task menu.
  **Cancel changes** restores the pose from before editing. **Reset draft to
  prefab** previews the original offset; Save is still required to keep it.

Alignment is local to the virtual overlay. Planning, execution, controller target
capture and target rotation are blocked while editing; existing plans are
invalidated. Saving or cancelling requires a new plan before execution. Joint
feedback and the chosen tool are retained.

The saved JSON contains the base pose relative to the QR, including its full
quaternion. It never stores a session's world-space origin. A matching profile
loads on future scans and app restarts, including when the XR origin changes.
Profiles are per device, robot profile ID and exact QR payload; the configured
printed marker size must also match. They are shared by the robot's tool choices.
The normal location is `Application.persistentDataPath/RobotCalibrations/` and
needs no administrator rights or machine-wide configuration. Clearing the app's
data also clears these local profiles. Invalid saved data falls back visibly to
the default, and a failed Save leaves the draft open for retry or cancellation.

## Tool menu and matching the running ROS model

`UR16eInteraction` now creates a world-space **Robot tool** menu with **No tool**,
**Kenos**, and **Kenos SL**. Aim the controller ray at a choice and press the
trigger. Selection updates the robot and TCP immediately; it also clears old
trajectory previews and invalidates previous planning. The menu starts near the
camera and stays where it is placed. **Recenter menu** brings it back
into view immediately. Selecting a button does not also place the robot or capture
a target.

### Moving the menu with the controller

The initial/recentered menu is about 1 m away, 20 degrees to the right and 15 cm
below eye height. Aim the controller laser at its grab bar, hold the trigger,
move or aim the controller to reposition the menu, then release. The menu keeps
that world position when the viewer walks or turns; it always faces the viewer
and stays upright. **Recenter menu** explicitly returns it to the initial offset.

The grab bar is separate from the buttons, so button clicks retain their usual
behavior. Menu grabbing suppresses robot placement and target capture. Focus
loss, pause or disabling the menu cancels an unfinished grab. QR placement and
realignment retain the viewer's chosen menu position.

The task view includes **Ghost path thickness** below Plan/Execute. Its horizontal
laser slider adjusts the shared TCP/MTC trajectory line from **5 to 50 mm** and
shows the current width in millimetres (the existing default is **25 mm**).
The old standalone `TrajectorySizeCanvas` is inactive in `UR16eInteraction`.
The separate 10/20/80% ghost-count buttons retain their existing behavior.

The offline choices come from the expanded ROS Xacros, including the stored arm
calibration. `RosRobotRig` applies the same joint origins, axes, fixed frames and
tool-to-TCP chain to both the live replica and trajectory ghosts. Joint feedback
is matched by name, so MiR joints and reordered feedback do not change the arm
mapping. The display uses measured joint positions immediately, with no gravity,
contact forces or keyboard controllers moving the replica independently.
Trajectory lines pass through the actual TCP, rather than the flange.

ROS tool selection remains a startup setting. The Unity menu changes the local
visual; restart ROS with the same `tool` choice to change the running robot model.
Unity reads `robot_description` using the read-only `GetParameters` services of:

- `/robot_state_publisher/get_parameters`
- `/move_group/get_parameters`
- `/mtc_test_task_node/get_parameters`

These service names are configurable on `ToolSelectionController`. Parameter
reads work for a late-joining headset; the installed ROS-TCP endpoint's volatile
topic subscriber cannot reliably receive an already-published description.
The robot-state publisher's actual description supplies custom calibration,
mounting, visual-origin and TCP parameters. Unity compares every arm/tool link,
joint axis/limit and visual/collision geometry against MoveIt and MTC. Planning
and execution require fresh matching descriptions from all three consumers.
The menu displays a mismatch or waiting status when this cannot be established.
**No tool** supports a TCP at the flange; it has no vacuum pick/place capability.

The Kenos SL link transforms match the ROS model, including the 34 mm lateral
contact-centre offset. Matching software does not measure the physical adapter:
its 60.25 mm height, 50 mm radius and zero clocking remain the ROS model's
provisional mounting assumptions. Calibrate those in ROS; the connected Unity
replica then uses the actual expanded values without a second set of offsets.

Kenos SL also loads the detailed supplied STEP geometry from the preconverted
`RobotModels/KenosSLVisual.bytes` resource. This mesh is local to
`kenos_sl_base`; it changes no mounting, link or TCP transform. ROS retains its
simplified box/cylinder collision envelopes. The 24 mm foam contact centre is
Unity-local `(0, 0.08975, 0.034)` metres, exactly the basis-converted ROS default
TCP. The adapter cylinder follows the ROS mount dimensions separately.
`KenosSLVisualProvenance.json` records the source STEP hash and conversion.
The converter under `Tools/RobotFrames` can reproduce the asset; the application
needs no CAD kernel, importer licence or additional package at runtime.

## Controller target preview in UR16eInteraction

The world-space **Robot tool** menu selects **No tool**, **Kenos** or **Kenos SL**.
This changes the local visual; ROS selects its mounted tool at startup. Before
planning is permitted, Unity reads `robot_description` independently from the
robot-state publisher, MoveGroup and MTC through their read-only GetParameters
services. Their complete calibrated arm/tool transforms, axes, limits and
geometry must agree. An actual shared custom model becomes the visual authority;
the catalog is the offline default. A different local selection, mismatched or
expired descriptions, disconnection, or failed visual load blocks commands.
No-tool has a valid flange TCP but does not offer vacuum pick/place.

The service names default to `/robot_state_publisher/get_parameters`,
`/move_group/get_parameters` and `/mtc_test_task_node/get_parameters`. Configure
these in the controller when ROS nodes are renamed. The installed ROS-TCP
Endpoint ROS2v0.7.0 subscriber uses volatile QoS, so subscribing late to the
latched `robot_description` topic alone is insufficient. Pending inspection
requests expire and can retry; stale callback generations cannot restore old
readiness. Selecting a different tool does not reconfigure the ROS robot.

The scene now starts in **TCP Preview** mode on `URInputManager`:

- Scan the existing QR marker, then press the trigger to place the robot using
  the unchanged QR-to-base calibration embedded in the robot prefab.
- A translucent cyan copy of the selected tool follows the controller pointer.
  It uses the same TCP-relative meshes and transforms as the robot, including
  the Kenos SL CAD. The tool's contact face is at the requested TCP; its outward
  **physical TCP +Z direction** follows the pointer's forward direction.
- Hold the bumper to capture green target A; double-press it to capture orange
  target B. Captured markers stay in tracking space, including when the XR Origin
  is translated or rotated. Labels show coordinates in `ur_base_link` in metres
  once the robot has been placed. These are local previews; no motion is sent.
- Before capture, the live cyan tool shows the pose that will be saved.
  After capture, aim at the tool or a curved arrow and hold the trigger:
  turn the controller while holding the tool for free rotation, or drag the
  orange tilt / violet twist arrows around their circles. Release to keep the
  orientation. The target point stays fixed. The arrows brighten on hover/select.
  Tilt is around target +X and twist around target +Z in ROS tool coordinates;
  free rotation can set any orientation. The cyan preview hides while a handle
  is hovered/held. Bumper capture and robot placement are suppressed while
  editing, so selecting a handle does not also replace a target or place a robot.
  Export reads the edited orientation directly.
- The task menu has separate **Plan TCP A/B** and **Plan MTC Pick/Place** modes.
  Both capture tool poses with the same controller convention. TCP A/B shows
  tool geometry and constrains the complete pose. MTC shows contact balls and
  normal arrows, and captures desired pick and
  place TCP contact points and normals; ROS samples twelve rotations around
  tool +Z independently at pick and place. The trajectory preview shows the
  actual chosen rotation. The internal virtual object remains 100 x 100 x 4 mm;
  these controls do not infer a scanned object's shape.
- Hover a captured reference and press the bumper to open its
  contextual menu. For MTC targets, **+Z 5 mm** pushes the contact point through
  the suction face and **-Z 5 mm** backs off. Each click moves exactly 5 mm along
  the tool's current normal. **Reset depth** undoes these translations, including
  when the normal was changed between steps. Recapturing a target resets its
  depth baseline. Close the menu to rotate the reference again. Opening a context
  menu suppresses pose capture;
  focus loss, pause or lost controller tracking closes it and clears input.
  Editing either target invalidates the previous plan, so Plan is required
  again before the separate Execute action.

The coordinate chain is:

`OpenXR tracking pose -> XR Origin Camera Offset -> Unity world -> inverse robot base_link pose -> ROS FLU`

The XR camera transform is not the tracking origin. The QR detector and controller
use the same origin transformation, applied once. Robot and captured targets share
that tracking-space parent. All transforms must preserve metres at unit scale;
invalid calibration or poses cannot produce a ROS target.
The QR-derived base pose is retained independently of the physics-driven robot
Transform; articulation-root teleports keep the visual robot aligned when its
tracking-space parent moves.

This calibration belongs to the current OpenXR session. A native reference-space
reset or headset relocalization is not the same as moving the XR Origin
Transform: cached tracking coordinates can then change meaning. Start a fresh
placement session and scan the QR again after such a reset; the stored QR pose
is not a persistent spatial anchor.

For a teach-pendant comparison, use the correct base frame: UR's `base` and ROS
`base_link` share a position but differ by 180 degrees around Z. The target
labels here use the imported robot's `ur_base_link`, not pendant `Base`.

ROSGeometry already converts position `(x,y,z)` to `(z,-x,y)`. Its full orientation
basis change means ROS tool +Z corresponds to a canonical Unity pose's local +Y.
Both capture modes apply the same fixed 90-degree local-X alignment to the
controller pointer orientation, so the physical TCP normal follows the pointer's
forward direction.

`TcpTargetPreview.TryGetCapturedRosPose` exports a canonical TCP `PoseStamped`
without publishing it. Its zero timestamp requests the latest TF for the static
robot-base target and avoids depending on headset/ROS clock synchronization.
The current client uses the plan-only TCP A/B interface and separately identified
Execute action. MTC tool contacts use **`/pick_place_tcp_target`**, gated by the
ROS node's `pick_place_tcp_planning_enabled` capability. The old
`/pick_place_poses_target` endpoint retains its legacy object-surface contract;
the new client never silently falls back to it. An older backend must be updated
before the new MTC mode can plan. The older `/tcp_pose_target` immediate-motion
interface is not used by these controls.

The accompanying ROS execution guard fix validates unique fixed-transform
**source** frame names (`header.frame_id`), rather than requiring unique
`child_frame_id` values: MoveIt stores every fixed transform with the same
planning-frame target. It still rejects invalid frame names and changes to
transforms referenced by collision geometry, while ignoring irrelevant live
controller-frame jitter. Source changes and offline checks do not update the
running Nordbo container; deploying that backend and rebuilding the Unity app
are separate delivery steps.

The preview does not measure a physical tool TCP. Set measured tool dimensions
in ROS and regenerate the offline catalog; connected Unity uses the actual ROS
description as described above.

## Model comparison

### Calibrated three-tool catalog

`Assets/Resources/RobotModels/Models.json` is generated from actual expansion of
the current ROS composition Xacro for `none`, `kenos`, and `kenos_sl`, explicitly
using the stored `ur16e_calibration.yaml`. It includes every link descending
from `ur_base_link`, six moving-joint origins/axes/limits, all fixed frame-only
links, and each visual/collision origin and geometry. The root is deliberately
the arm base; MiR/mount transforms are outside this catalog.

The runtime data uses ROS FLU, metres, XYZW unit quaternions and radians. Joint
motion composes as `parent * joint_origin * joint_axis_rotation`. Unity must
convert both position and orientation through the ROS/Unity basis. Each tool
has 21 complete FK reference configurations: zero, home, bent, six independent
quarter turns and twelve deterministic random configurations. These references
cover 903 individual link poses, including `ur_base`, `ur_base_link_inertia`,
`ur_ft_frame`, `ur_flange`, `ur_tool0`, and the actual unprefixed ROS `tcp`.

The exporter retains the three fully expanded descriptions beside the catalog
as `none.urdf.txt`, `kenos.urdf.txt` and `kenos_sl.urdf.txt`. Source-file SHA-256
hashes include the real ROS macros/defaults, calibration, consumed upstream UR
Xacro/YAML, control-wrapper generator and local tool mesh bytes. Upstream
dependency revisions are explicit. Sparse upstream checkouts may omit mesh
bytes; `unavailableMeshUris` reports those paths rather than inventing hashes.
These snapshots identify an offline source model, while querying the active
RSP `robot_description` is necessary to establish deployed model parity.

Regenerate without starting ROS nodes, using the existing dependency source
layout (`xacro`, `ament_index`, `ur_description`, `ur_driver`) and Python with
Xacro, PyYAML and `ament_index_python` on its import path:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$DEPS/xacro:$DEPS/ament_index/ament_index_python" \
python3 -B Tools/RobotFrames/export_ros_models.py \
  --ros-root /path/to/UR16UnityControl --dependency-root "$DEPS"
```

The generator creates a temporary installed-style control wrapper with the
ROS repository's own generator and the selected driver's actual signature.
Its temporary ament package index points to the real source packages. It does
not patch macros or synthesize an arm from the old Unity export. Source
checkouts remain read-only, and no ROS process or robot command runs.

`verify_runtime_models.py --actual <runtime-dump.json>` checks every model,
configuration and link against these expanded-ROS goldens. Its input is
`{models:[{tool,cases:[{name,links:[{name,position,rotation}]}]}]}`, with actual
Unity runtime poses converted back to base-relative ROS FLU. It rejects missing
or extra links/configurations/tools, invalid poses and intermediate arm errors
even when the final TCP coincides. Default tolerances are 0.01 mm and 0.001
degrees. Quaternion signs are equivalent. The script writes a structured report
and exits nonzero on a mismatch. A dump copied from the catalog is a verifier
self-check only, not runtime validation; actual serialized-prefab tests must
exercise Unity articulation physics before recording transforms.

No-tool ROS attaches `tcp` at the flange identity, not at `ur_tool0`. FlexiGrip
and Safe&Light have different fixed chains and TCP rotations. Safe&Light keeps
its 34 mm eccentric working-point offset; its current nominal mount dimensions
remain provisional physical measurements even when ROS/Unity models match.

`unity_validation_summary.json` preserves the measured 2026-10-22 isolated Unity
6000.0.84f1 result: **160/160 tests passed**, including 1,806 actual serialized
main/transparent-prefab link poses after native articulation physics, active hot
tool switching, arbitrary base placement, added fixed frames, CAD geometry and
transparent ghost materials. Maximum observed position difference from expanded
ROS goldens was **0.002027 mm**, and maximum basis-vector difference was
`2.58035448e-6`. The summary hashes every validated source/asset/fixture and
records the isolated workflow stubs and untested runtime boundaries.

The DTO tests use CDR bytes generated by installed ROS2 `rcl_interfaces` and
`rclpy.serialization.serialize_message`, including Unicode XML, all parameter
arrays and integer extremes. Only schema-proven CDR alignment gaps are masked,
because Fast-CDR does not guarantee their padding contents. Neither fixture
generation nor these tests initializes ROS or makes service/robot calls. Actual
service transport, headset interaction, planning, physical calibration and
hardware operation remain outside this validation.

`prepare_unity_validation.py` recreates the test project under ignored
`Temp/RosRigValidation` by copying unchanged prefabs and their asset dependency
closure, current helper/runtime/test files and cached packages. It never opens
the active project. `record_unity_validation.py` records a passing test run and
rejects a current source file that differs from its validated copy. Large logs
and NUnit XML stay in Temp; the compact measured summary is retained here.

The older comparison below is retained as an offline diagnostic of the original
nominal Unity export. Runtime geometry must use the new expanded calibrated
catalog; the old calibration-proxy report is not the model authority.

### Original export diagnostic

Run this standard-library-only Python script from the Unity project root:

```powershell
python Tools/RobotFrames/check_robot_frames.py --ros-root C:/Users/aca/source/UR16UnityControl
```

It reads the checked-in combined Unity URDF, the ROS Kenos Xacro/default TCP
arguments, and the stored arm calibration. It writes an inspectable
`reference_comparison.json` beside the script, including source hashes, each
fixed tool transform, six arm-origin differences, and five forward-kinematics
comparisons. It contacts no robot and changes no Unity or ROS source files.

The FK comparison isolates arm calibration using the fixed arm transforms and
joint axes from the Unity export. It is a calibrated proxy, not an expansion of
the deployed ROS model. It does not inspect the serialized Unity prefab, prove
reachability, or establish a maximum error over every robot configuration.

The legacy combined Unity URDF checked by `check_robot_frames.py` contains the
**old/default Kenos model**. The runtime catalog above supersedes its nominal
arm calibration and supplies all three tools. The replacement Kenos
variant needs its mounting geometry and working TCP measured. Matching this old
model does not establish that it matches the replacement. No replacement tool
dimensions are applied to geometry or runtime settings; the optional estimate
below stays separate in the report.

The owner's provisional description is a suction pad, approximately **150 mm
thick**, with **no rotation**. To reproduce the checked-in report's separate,
unapplied estimate:

```powershell
python Tools/RobotFrames/check_robot_frames.py --ros-root C:/Users/aca/source/UR16UnityControl --estimated-tool0-depth-mm 150
```

This explicitly assumes that 150 mm is the distance from `ur_tool0` to the
working TCP along **tool0 +Z**, with identical orientation to `ur_tool0`. The
checker composes that through the actual combined URDF's fixed
`ur_flange -> ur_tool0` transform. The estimate appears only under
`provisional_tcp_estimate` in the report, marked `UNMEASURED_ESTIMATE_NOT_APPLIED`.
It changes neither the old-model comparison nor any geometry or runtime setting.

Body thickness may omit an adapter or mounting plate, and a suction pad can
compress at contact. Measure the flange-to-working-point offset before using
it. Teach-pendant **RX/RY/RZ are an axis-angle rotation vector**; URDF **rpy is
fixed-axis roll/pitch/yaw**. They are not interchangeable for nonzero values.
The identity rotation is the only equivalence assumed here.

ROS configuration locations, relative to the external ROS repository:

- Select `none` or `kenos` with the `tool` argument in
  `ros_ws/src/ur16e_bringup/launch/ur16e_start.launch.py`. Python bringup defaults
  to `none`; the separate `ros_ws/run_bringup.sh` defaults to `kenos`.
- `tool_tcp_xyz` and `tool_tcp_rpy` defaults are in
  `ros_ws/src/ur16e_mir250_description/urdf/mir250_ur16e.urdf.xacro`. For Kenos,
  these describe the TCP **relative to `kenos_base`**, not the flange or wrist.
- Mount/body transforms and meshes are in
  `ros_ws/src/kenos-flexigrip/urdf/kenos.urdf.xacro`.
- Stored arm calibration is in
  `ros_ws/src/ur16e_mir250_control/config/ur16e_calibration.yaml`. Primary
  bringup forwards it through `kinematics_params_file` to control, MoveGroup and
  MTC. The URDF Xacro argument itself is named `kinematics_params`.
- `ros_ws/src/ur16e_mir250_description/scripts/generate_urdf.sh` accepts extra
  Xacro arguments such as `kinematics_params:=...`, `tool_tcp_xyz:=...`, and
  `tool_tcp_rpy:=...` for a later export. Its defaults use nominal arm geometry.

The current primary ROS launch files do not expose or forward custom TCP
arguments to every model consumer. Editing only an export command would change
the Unity export while leaving the primary ROS runtime defaults unchanged.
Any measured tool update needs matching control, planning, MTC and Unity models.

Run the focused mathematical/parser checks with:

```powershell
python -m unittest discover -s Tools/RobotFrames -p "test_*.py"
```

## Detailed Kenos SL visual from the supplied STEP

`Assets/Resources/RobotModels/KenosSLVisual.bytes` contains the complete supplied
Piab KSL assembly tessellated into `kenos_sl_base` local Unity RUF coordinates in
metres. This optional visual replaces no ROS joints, TCP, calibration or collision
geometry. The mounting changer is absent from the supplied CAD and retains its
separate ROS representation. The inspection OBJ lives outside Resources at
`Assets/Models/KenosSL/KenosSLVisual.obj`.

The STEP root mounting centre is `(0, -34, 31.75)` mm. The conversion is
`Unity = (-STEP.X, 31.75 - STEP.Z, STEP.Y + 34) / 1000`; triangle winding reverses
once for the change of handedness. The foam contact centre `(0, 0, -58)` mm becomes
`(0, 0.08975, 0.034)` metres. This places the supplied geometry without changing
the robot's link transforms. The provenance JSON beside the binary records source
and output hashes, package versions, all nine solid bounds and conversion settings.

Regenerate with Python 3.12 and `cadquery-ocp==8.0.1.1.0` plus its declared
dependencies (`cadquery-ocp-proxy==8.0.1.1.0`, `vtk==9.6.2`):

```powershell
python Tools/RobotFrames/convert_kenos_sl_step.py --input "C:/path/to/KSL.240.120.N241.CVL.S1.1STH.X.stp"
```

The converter accepts the exact owner-supplied source hash only. It can load
isolated packages unpacked under `Temp/KenosSlCad/packages` (`--ocp-path` overrides
this). No administrator rights or machine-wide installation are needed. At
0.3 mm linear and 0.3 radian angular deflection, it exports 24,569 vertices and
26,114 triangles. Nine zero-area CAD pole triangles are omitted. The STEP reader
reports one unresolved reference, while all nine valid assembly solids and 794
faces transfer and tessellate.

Runtime loading uses the binary, bypassing OBJ importer axis conversion. Its exact
layout is ASCII `KSL1`, little-endian int32 vertex count and index count, float32
XYZ positions, then int32 indices. Mesh normals are recalculated in Unity. The
converter validates the mounting/contact references, foam and overall bounds,
finite positions, triangle winding, index range and binary round trip.

## QR alignment audit and rescan

`qr_validation_summary.json` records the 15 passing actual-component Unity EditMode
tests, source hashes and remaining hardware limits. Three tests instantiate the
active robot prefab at different QR/origin rotations, apply the calibrated ROS
model and step native articulation physics. The base retains exactly one prefab
QR extrinsic, within 0.00002 m and 0.00002 basis-vector error. Reanchoring retains
the same robot, selected model and joint feedback; Rescan immediately invalidates
coordinate calibration and old plans.

The unchanged prefab QR-to-base pose is `(-0.419, 0.230, 0.505)` metres with
`Rx(-90 degrees)`. The historical `ControllerBox/QR1` geometry is consistent with
that inverse for a face-up QR, within its rounded measurements; the name `QR1`
does not encode the physical marker payload. An empty expected payload now
requires one unambiguous QR and pins its decoded identity until Rescan. A lost,
invalid or discontinuous selected observation resets the averaging window.
Normal asynchronous OpenXR `Pending` snapshots do not count as lost observations.

The photographed horizontal offset has not been physically resolved. An in-plane
180-degree marker-axis difference would preserve height while reversing the
horizontal lever arm, but no such rotation is applied without measurement.
The visible red/green/blue QR axes and one-shot payload, size, tracking/world pose
and base-calibration logs provide the evidence for the next headset scan.
After the SDK basis conversion, blue QR +Z points into the printed face.

SDK 2.6.0 exposes no public QR location-validity flags: its `GetUnityPose` may
return identity or partially filled poses on native location failure, and its QR
reprojection-error field is always zero. A genuine identity pose remains valid.
Consecutive-observation and spread checks reduce contamination but cannot
distinguish a persistent native failure from a legitimate identity through that
public API. These tests start no OpenXR session, ROS connection or robot command.
