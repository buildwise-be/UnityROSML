# Item-surface picking and Cartesian descent speed

In Simple Pick and Place, select **MTC pick / place**, then **Pick settings**.
Choose **5 mm/s** (default), **10 mm/s**, or **20 mm/s** for the pick descent.
Changing the speed, item or pickup method invalidates the reviewed plan and
repeat eligibility. Plan again with the new settings before Execute.

**Center** retains the existing contact-point workflow: hold the bumper to
capture Pick and double press it to capture Place. The picked point can be away
from the actual object's centre; this method does not measure the surrounding
object surface. New requests use the selected catalog item's nominal box.

For **Item surface**, select **Tile** or **Brick**. A floating item illustration
appears in front of the viewer, separate from the menu. Its orange sphere and
number identify the next top corner. Touch that physical corner with the
controller's calibrated contact tip and press Trigger. Capture three
consecutive corners: P0, P1 along the item's length, then P2 along its width.
Undo, Recapture and Cancel are available. The guide never collides with the
controller or becomes a planning-scene object.

The three contacts fit a proper rigid transform without changing item size.
The fitted top-face centre becomes Pick, with tool +Z directed into the face.
The box's outward normal and length/width axes remain separate from the tool's
free wrist twist. The resulting measured pick is read-only; switch to Center
to capture a manually chosen contact. Place remains the usual independent
double-bumper contact. A canceled or rejected recapture keeps the previous pick
and place. Tracking resets, suspension or changed robot alignment/model require
a new valid surface capture. Rigid XR Origin movement preserves tracking-local
contacts. The object uses the same configured physical controller-tip offset
as five-point robot registration.

After the third corner is accepted, the full-size item appears at the fitted
pose with the same cyan visible-contour style as the robot. Its top face is
the measured surface and its thickness extends below that face. The item
outline does not require enabling the robot's outline checkbox. An accepted
item remains during a replacement capture or cancellation, and hides when the
selected method/item, workflow visibility or alignment/tracking context no
longer applies. It represents the captured pick location; it does not track
the physical item as the robot moves it.

The fitted rectangle must agree with nominal length, width and diagonal within
15 mm, with normalized edge dot product no greater than 0.15 in magnitude.
The rigid fit allows at most 10 mm RMS and 15 mm worst-point residual. Rejected
measurements do not silently resize the object or replace an accepted pick.

| Item | Length × width × thickness | Assumed solid density | Voids | Estimated mass |
| --- | --- | --- | --- | --- |
| Ceramic tile | 200 × 200 × 7 mm | 2,400 kg/m³ | 0% | 0.672 kg |
| Clay brick | 280 × 185 × 135 mm | 2,000 kg/m³ | 20% | 11.1888 kg |

Mass is `length × width × thickness × assumed density × (1 − void fraction)`.
These are provisional material estimates rather than measured item weights.
The nominal tile density is within the 2,380–2,450 kg/m³ range reported by a
[porcelain-tile EPD](https://www.environdec.com/library/epd6762). The rounded
solid-clay assumption is close to the 1,980 kg/m³ clay-brick example in the
[ASHRAE material-property tables](https://handbook.ashrae.org/Handbooks/F25/SI/F25_Ch26/F25_Ch26_si.aspx).
The full outside box is used for collision checking because the locations and
shapes of the brick's holes are unspecified. Estimated mass is descriptive
metadata; it does not automatically change the robot's configured payload.

## Unity–ROS request contract

The additive topic `/pick_place_task_target` uses
`ur16e_interfaces/msg/PickPlaceTask`. The legacy `PickPlacePoses` message and its
existing topics retain their original wire format.

The request contains the two contact poses, `cartesian_descent_speed_mps`
(`0.005`, `0.010` or `0.020`), `item_type` (`tile` or `brick`),
`pick_from_surface`, and exactly three `geometry_msgs/Point` slots. Surface
corners are in the contact header's static `ur_base_link` frame. Canonical
object axes are ROS +X along length, +Y along width and +Z outward from the top:
P0 = (−L/2, −W/2, 0), P1 = (+L/2, −W/2, 0), P2 = (+L/2, +W/2, 0).
Center-mode requests leave the ignored corner slots zeroed.

Unity requires the read-only backend capability
`pick_place_task_planning_enabled` before sending this message. There is no
fallback that silently discards speed or geometry on older ROS installations.
ROS independently validates the preset, speed, contacts and corners, derives
the item pose and places the collision-box centre half a thickness below the
fitted top face. The selected descent speed is copied into both the planned
Cartesian timing and the immutable execution policy of the reviewed plan.
Repeated execution uses that frozen choice.

Selectable descent works with either the ordinary fixed-distance Cartesian
approach or the optional commissioned guarded-contact approach. ROS retimes
the ordinary pick approach at the selected TCP speed; joint-velocity scaling
alone is not treated as a millimetres per second setting. With guarded contact
enabled, the selected speed applies to its prepared descent instead. The UI
does not enable that mode, change its commissioning flags, or modify startup
force/contact parameters. Place approach and legacy requests retain their
existing timing.

The earlier implementation rejected this request when guarded contact was
disabled with "Selectable Cartesian descent requires enabled, commissioned
guarded contact". That was an unnecessary dependency in the new endpoint.
Updating the MTC node supplies the ordinary approach timing fix; changing
commissioning flags is not the remedy.

Both updated Unity code and the new ROS interface/node must be built together.
The ROS-TCP endpoint must run with the rebuilt message overlay. Source changes,
isolated builds and synthetic tests do not update an installed headset app or
activate the production stack.

## Validation

### Compatibility and item-outline repair — 2026-11-06

The final full Unity graphics suite passes 749/751 checks with no skips. The
same two failures reproduce on the unchanged baseline (735/737):
`ToolCalibrationMenuTests.AlignmentActionsFitTheExistingPanelAndOnlyTaskViewStartsVisible`
and `WorkflowServiceLifecycleTests.ServiceReusesTheConnectorPreservesHudEditsAndRecoversAReplacement`.
The 305 targeted workflow checks and 62 focused contour/graphics checks pass.
Runtime compilation has zero errors. Tests verify fitted size/pose, independent
item contours, recapture/context behavior, transparent grazing-angle faces,
orthographic views and real depth boundaries through both installed renderers.

The ROS runtime builds in an existing local WSL image with networking disabled.
All 105 relevant C++ cases and nine synthetic node replays pass. Ordinary
Execute/repeat trajectories retain 4.995, 9.990 and 19.980 mm/s maxima for the
three choices. The tilted brick and optional guarded path also pass. Full
CTest remains 19/20 with the unchanged scene-effects failure. The local
dependency image differs from the current production image; these checks do
not establish physical speed or headset appearance. The running production
stack was not changed, and the updated headset app was not installed.

See [repair evidence](PickSurfaceRepairValidation/validation_summary.json),
[ROS repair evidence](PickSurfaceRepairValidation/backend-summary.json),
[tile outline](PickSurfaceRepairValidation/tile-outline-moved-camera.png) and
[brick outline](PickSurfaceRepairValidation/brick-outline.png).
This repair changes no ROS message schema; update the MTC executable for speed
compatibility and rebuild the Unity app for accepted-item contours.

### Initial implementation — 2026-11-05

All 280 selected Unity checks pass, including point capture/cancellation,
calibration and model invalidation, selected-speed export, older-backend
rejection, existing input/planning regressions and exact canonical CDR byte
comparisons against four ROS-generated fixtures. Runtime compilation has zero
errors. Six item/corner guide states render without text overflow.

The ROS interfaces and complete task node build against the production
dependency image. All 104 relevant C++ cases pass; full CTest passes 19/20
targets with the documented unchanged scene-effects assertion. Four isolated
new-topic planning replays verify all three speeds and the tilted brick's live
and frozen scene pose/dimensions. A simulated Execute and repeat retain the
selected 20 mm/s despite a different startup speed. Invalid speed/corner
requests produce no scene updates. These are network-isolated synthetic checks.

See [validation evidence](PickSurfaceValidation/validation_summary.json),
[backend evidence](PickSurfaceValidation/backend-summary.json), and
[the floating brick guide preview](PickSurfaceValidation/brick-corner-3.png).
The preview uses synthetic contacts; no headset installation or production
stack activation was performed.
