# Fixture regrasp and tile batches in Unity

The **Repetitive Pick and Place** scene implements the Unity side of the ROS
`codex/fixture-tiling` workflow. The app starts at **Welcome**, where the user
chooses **Simple Pick and Place** or **Repetitive Pick and Place**. Each opens
its own scene; the simple scene retains TCP A/B and single-item MTC controls.
The repetitive scene offers floor placement and the fixture controller with its
dedicated setup and review controls. Choose **Fixture setup** for the fixture
workflow described here. The build contains these three scenes in that order.

The fixture workflow accepts square tiles, a horizontal aligned stack and
centred suction on the large top face. A new setup starts with a 200 × 200 × 7 mm
tile. Enter the measured side and thickness before capturing its top face.
Previously saved setup values are loaded from the headset.

The **fixture board** is the physical sloped board where the robot releases a
tile, lets it settle against the stops, and picks it up again for final placement.
Its top surface, thickness and stop corner are captured separately.

## Configure and capture

1. Choose **Repetitive Pick and Place** on Welcome. Align the XR robot with the
   real robot and select the matching ROS suction tool. Use **Fixture setup**
   to open the fixture setup pages.
2. On **Tile dimensions**, enter the measured square side, thickness, source
   count, job count and stack pitch. The whole source stack is selected for
   placement by default. Changing the source count updates the job count while
   the whole stack is selected. **Place entire stack** selects all source tiles
   again after choosing a smaller job. Zero pitch uses thickness. Length
   steppers have 0.1, 1, 10 and 100 mm increments.
3. On **Source stack**, choose either **Capture stack top centre** or **Capture
   top tile · 3 corners**. For the centre method, touch the centre of the upper
   face and press trigger. It keeps the current source yaw, which starts at zero.
   For three corners, the main menu hides while the item guide is visible. Touch
   three consecutive top corners and press trigger at each, following P0, P1
   along one edge and P2 along the next edge counterclockwise. The fit uses the
   configured tile dimensions and sets the source centre and edge direction.
   After the third accepted contact, the setup menu returns. **OK** cancels the
   current capture and returns to setup. Both methods show a cyan pick-centre
   cross and dot on the green source stack. Inspect that centre and use the yaw
   adjustments to align the ghost edges. Source yaw is sent to ROS.
4. On **Fixture board**, capture the top perimeter as P00, P10, P11, P01,
   ordered so the top normal points upward. Enter board thickness and edge
   margin. Choose **Fit fixture board rectangle** and inspect the fitted ghost and
   reported maximum corner adjustment. Fitting rejects an adjustment above
   10 mm; recapture inaccurate or incorrectly ordered corners.
5. On **Fixture stop corner**, capture the inner stop corner on the supporting
   surface, then points along both arms into the receiving area. The measured
   origin/axes are projected onto the fitted board plane; the axes must have
   perpendicular intent and the correct order. Gravity must pull toward both
   stops. The corner is an alignment reference; its thin metal geometry is
   omitted from ROS collisions by agreement.
6. **Save fixture to ROS** validates and persists the calibration through
   `/save_fixture_calibration`. Inspect the result, then restart/reload MTC to
   activate the saved file. Saving does not change the running scene. When
   replacing a loaded calibration, an existing `fixture_plank` may need explicit
   scene reset/replacement, as described in the ROS setup guide.
7. On **Release region**, capture its centre on the fixture board, or adjust X/Y from
   the alignment corner. Width/depth bound tile centres. Clearance is measured
   from the tile bottom along the board normal. ROS checks complete tile
   footprints, release alternatives and the settling envelope.
8. On **Final placement**, capture the first tile's corner on its final
   **top face**, then +X and +Y along the pattern edges. The status and checkmarks
   confirm each captured point. The first corner leaves an orange marker,
   followed by a red +X marker and line, then a green +Y marker and line.
   The complete orange tile pattern appears when all three points define a
   valid frame. Enter columns, mortar gap and alternating row shift. Half-pitch
   staggering includes the gap. Count and columns determine rows and a possible
   partial last row.
9. Set withdrawal distance, the additional wait **after withdrawal**, and
   optional return to the configured cycle home. The fixture assumes the tile
   has seated after the delay; it has no alignment sensing.

Every capture is selected in the menu and completed with the trigger outside
the menu. Trigger does not place the robot while fixture mode is selected.
The app stores completed setup in `fixture-tiling.json` in Unity's per-user app
data directory, and stores fixture calibration on ROS through the save service.
Persistence requires the fixture-to-robot relationship to remain fixed.

The **Welcome** button in the workflow header returns to the welcome scene.
Scene changes discard local review permission and require fresh XR robot
alignment in the next workflow. Saved calibration profiles and fixture setup
remain available. Navigation is blocked while an execution outcome is pending,
including an unknown acknowledgement outcome, so scene unloading cannot discard
the active command's feedback. One established ROS connection is retained
across scene changes; scene-owned subscriptions are removed on unload.

## Plan, review and execute

**Plan preview** queries the running read-only `fixture.*` parameters and checks
they match the local calibration before publishing one new request ID. All
coordinates are static, zero-timestamp `ur_base_link` poses in metres. Source
and destination are item frames; their +Z points out of the item face. They
are not TCP frames and receive no Unity tool offset.

Unity subscribes to `/mtc/fixture/status`, `/mtc/fixture/preview_scene` and
`/mtc/fixture/solution`. The ready status must match the current request, and
its plan ID must match both `preview_scene.name` and `solution.task_id`.
Delivery order does not matter. The separate uncorrelated
`/mtc/fixture/planned_trajectory` topic cannot arm execution: Unity derives
the robot path from the identified ordered solution instead.

Review the robot ghost path using the existing playback controls. On **Review
task steps**, step through the initial inventory and the ordered motion,
attachment, release, settling and placement scene effects. Step selection also
seeks the robot ghost to that step's motion endpoint. The robot-only timeline
omits IO and waiting time; the step text exposes those markers. The scene review
is a model of the assumed fixture transition, not a gravity simulation.

Only a matching full scene, successful complete solution and displayed robot
path enable **Execute plan**. It publishes the exact plan ID once to
`/execute_fixture_plan`. A repeated click or late `ready` message cannot rearm
it. Missing acknowledgement retains the global command lock as an unknown
outcome; late matching terminal status still resolves it. Mode changes and
edits revoke review permission without discarding an active execution outcome.
Fixture batches do not support reviewed-plan repeat.

The scene viewer renders boxes, spheres, cylinders, cones and meshes, including
attached tile poses calculated using the reviewed joint states and robot model.
Unknown collision frames, malformed geometry, infinite planes and nonempty
octomaps block this preview; use finite support collision boxes for the current
workflow. Meshes are visualized from their actual vertices. Planned collision
checking remains ROS's responsibility.

After an execution attempt, inspect the physical inventory and ROS scene before
creating another job. Returning home does not replenish the source stack or
remove placed tiles.

## Message compatibility and offline verification

`RosMessageSchemas/manifest.json` records the exact schema closure from the
fixture-build image. The newer Jazzy `SolidPrimitive` and `Marker` schemas
contain fields absent from the bundled Unity connector; private embedded
serializers preserve those fields without replacing its global registrations.
Regenerate via `generate_fixture_preview_messages.ps1` after deliberately
updating the schema snapshot. New fixture task/service wire layouts mirror the
ROS message definitions.

Prepare an isolated project with the actual runtime, UI, messages and tests:

```powershell
& 'C:/Users/aca/AppData/Local/Programs/Python/Python312/python.exe' Tools/RobotFrames/prepare_fixture_tiling_validation.py
& 'C:/Program Files/Unity 6000.0.84f1/Editor/Unity.exe' -batchmode -force-d3d11 -projectPath 'C:/Users/aca/unity-source/UnityROSML/Library/FixtureTilingValidation' -runTests -testPlatform EditMode -testResults 'C:/Users/aca/unity-source/UnityROSML/Library/FixtureTilingValidation/results.xml' -logFile 'C:/Users/aca/unity-source/UnityROSML/Library/FixtureTilingValidation/tests.log'
```

The isolated ROS service throws before obtaining transport. Workflow tests
inject messages and override publication; they do not connect to ROS or operate
hardware. This is software validation. Device XR capture, live ROS-TCP delivery,
physical fixture calibration and robot execution require commissioning.
