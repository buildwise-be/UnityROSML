# Pick descent compatibility and accepted-item outline

The owner reports that the new 5/10/20 mm/s request is rejected with a guarded-contact commissioning error, and requests an outline of the real item after accepting its three top corners.

## Intent and constraints

- Keep 5 mm/s as the default and honor all three supported choices in the actual ROS trajectory, its Unity preview, and explicit repeats.
- Support the ordinary, non-force-monitored pick mode already used by the application. Preserve the existing guarded-contact mode when explicitly configured. Never enable or mark that mode commissioned automatically.
- Keep rejected requests side-effect free; preserve known item dimensions/orientation and legacy request behavior.
- Show the accepted, full-size item below its fitted top plane using the existing cyan, 2 px visible-contour renderer. The placed item is independent of the robot's outline toggle.
- Preserve an accepted item while replacement capture is pending or cancelled. Hide it when the selected item/method, calibration, model, or tracking context makes it inapplicable. Follow the existing pick-marker visibility/lifetime and do not imply live physical item tracking.
- Preserve the owner's unrelated ProjectSettings modification. No machine-wide installation or Windows administrator access. No production robot motion or automatic commissioning.

## Approaches

Simply removing the ROS rejection would leave joint-scaled descent that ignores the requested mm/s. Automatically enabling guarded contact would change the operator's execution policy. The chosen implementation approach is to retime the existing ordinary Cartesian approach at the requested TCP speed, preserving the configured guarded branch independently. Prepared plans retain the request metadata in either branch.

A separate wire-box style would differ from the requested robot style. Extend the existing contour draw registration to accept the fitted item's mesh independently of the robot toggle, keeping the shared shader, line width, private depth, camera availability, and fallback behavior.

The real pixel checks exposed a shared shader defect at grazing angles: ordinary depth slope across a continuous plane was classified as a discontinuity, filling the thin tile face. Correct that classification using local plane-depth slope while retaining silhouettes, creases, and genuine depth jumps. Validate perspective and orthographic views and preserve render-time XR view handling.

These are implementation decisions based on the owner's explicit request; they do not authorize production activation or claim physical speed verification.

## Validation

Reproduce disabled-guard rejection before changing runtime code. Check ordinary descent trajectories at 5/10/20 mm/s through planning and synthetic Execute/repeat, plus guarded-mode regression and invalid requests. For Unity, check fitted pose/dimensions, independent contours, recapture/cancel, selection/context invalidation, and cleanup. Render the accepted tile and brick with the real contour feature. Run the existing runtime compiler and relevant Unity/backend regression suites, retaining any baseline failures by name.
