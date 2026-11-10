# App scenes

The application build has exactly three scenes:

1. [Welcome](../../Assets/Scenes/Welcome.unity): the entry scene with **Simple
   Pick and Place** and **Repetitive Pick and Place** choices. It contains the
   shared XR rig and event system, with no robot controllers or ROS service.
2. [SimplePickAndPlace](../../Assets/Scenes/SimplePickAndPlace.unity): the
   existing TCP A/B and single-item MTC workflows, tool selection, robot
   alignment and playback. Fixture controls are unavailable here.
3. [RepetitivePickAndPlace](../../Assets/Scenes/RepetitivePickAndPlace.unity):
   fixture calibration, square-tile stack/pattern setup and complete-batch
   review/execution. Its serialized controller is on the input object; the
   simple planners and targets are absent.

The two task scenes derive from the validated V3 interaction setup and retain
its XR action references and robot prefabs. Legacy development/sample scene
assets remain outside the app's build list. `ProjectSettings.asset` is not part
of this change; [EditorBuildSettings](../../ProjectSettings/EditorBuildSettings.asset)
selects the three app scenes with Welcome at build index zero.
Open `Assets/Scenes/Welcome.unity` when starting this flow with the Editor's
Play button; built applications start there automatically.

Both workflow headers have a **Welcome** button. Navigation cancels target
interactions and clears scene-owned robot placement/readiness, while saved
calibration files remain available. Each destination workflow starts with
fresh XR alignment. A pending or unknown execution blocks navigation until
its terminal outcome arrives; it is never discarded by switching scenes.

ROSService retains one connector across workflows, without serialized duplicate
connector prefabs in the task scenes. Its initial address remains the former
interaction scene's `10.200.101.144:10000`; subsequent connection-HUD changes
are retained. Welcome does not create ROS transport on initial app launch.
Returning to Welcome retains an already-established connection. Legacy MTC,
TCP, fixture and joint-feedback subscriptions are removed with their owning
scene components.

See [fixture setup](FIXTURE_TILING.md) for the repetitive workflow.

Simple MTC pick/place has **Pick settings** for 5/10/20 mm/s descent and Center
or three-corner Item surface pickup. See [surface picking](PICK_SURFACE.md) for
the item presets, floating guide, measured frame and additive ROS contract.

While ROS is planning a simple pick/place, TCP A/B, or fixture batch, select
**Stop Planning** in the task menu. You can plan again once ROS releases the
planner. **Stopping…** may stay visible until the current solver or scene
operation returns. A stopped request cannot produce an executable preview.

Both task menus include force and torque magnitude gauges using the shared ROS
connection. Execution peak markers remain until a new Plan or Execute starts;
waiting/disconnected and stale states hide the live readings and retain peaks. See
[force telemetry](FORCE_TELEMETRY.md) for the topic, units and configuration.

## Offline checks

`prepare_scene_workflow_validation.py` copies actual runtime/UI/navigation and
the exact scene/build YAML snapshots into an isolated project under `Library`.
The manifest records source hashes and script/scene GUIDs. Static scene checks
verify build order, controller references, workflow restrictions and XR/ROS
wiring without running native XR components. Callback-only navigation tests
verify routing and execution guards; service lifecycle tests exercise actual
disabled connector components without starting a network connection.

Use `--stub-transport` for the existing broad workflow suite, or the default
actual-service harness for the focused scene/lifecycle checks. The explicit
`RobotAppSceneTests.RenderSceneWorkflowPreviews` export renders the welcome
screen and both task menus for layout inspection. Physical headset selection,
native XR scene transitions and live ROS-TCP delivery remain commissioning
checks.
