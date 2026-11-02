# Fifth registration contact: fixed base rear

The fifth contact is **Base rear**, appended after the existing Shoulder, Elbow,
Wrist 2 and Wrist 3 contacts. It is on the stationary gray base housing, upstream
of shoulder pan. It is not a blue cover on `ur_shoulder_link`.

The active ROS catalog has an empty `ur_base_link` visual and puts the base mesh
on its fixed child `ur_base_link_inertia`. That link's visual 0 references
`package://ur_description/meshes/ur10e/visual/base.dae`, also used by the UR16e
description. Its fixed-joint half-turn and visual half-turn cancel, so mesh
coordinates coincide with `ur_base_link` coordinates. ROS base -X is the rear.

The contact uses `eSeries_UR10e_010`, bound to `JointGrey-material`, in:

`Assets/Prefabs/ur16e_kenos/meshes/ur_description/meshes/ur10e/visual/base.dae`

Source SHA-256: `f39d15630605cb2959d33db68545c8e75048d67e88c592b752be0e77f4eee8d3`.

This mesh has gray housing and a black upper ring, with no blue cover. The blue
cap in `shoulder.dae` belongs to the moving shoulder and does not meet the
fixed-base requirement.

The gray housing spans approximately Z=0 to 84.948232 mm. The chosen contact is
at its mid-height, rounded to Z=42.474 mm, on the rear center line Y=0. A negative-X
ray through that line intersects the exterior CAD triangles at:

```text
mesh position (m): (-0.0867866605914, 0, 0.042474)
stored position:  (-0.086787,       0, 0.042474)
guide direction:  (-1,             0, 0)
```

The stored contact is within 0.00034 mm of the exterior CAD surface. The guide
identifies the gray housing's rear mid-height contact rather than claiming a
physical mark or cover-center feature exists there. Its location remains
inspector-editable for a measured physical reference.

`RobotCoverLandmarkTests.DefaultContactIsOnTheActualMeshOuterSurface` independently
loads the COLLADA node matrix, position arrays and triangle/polylist indices,
then checks that the stored contact lies on that exterior face within 0.02 mm.
Additional tests require the fifth point to remain fixed when every arm joint
changes, retain the four earlier contacts in their original order, and preserve
all five pairs in the registration fit and reported residuals.

`RobotCoverLandmarks.UpgradeDefaults` upgrades only untouched serialized legacy
four-contact defaults. It leaves custom arrays intact so a measured setup cannot
be silently replaced. A custom four-contact setup needs an explicit fifth
definition before the five-point workflow can proceed.

CAD agreement does not measure the real housing, the controller-tip offset or
headset tracking. Verify the selected rear mid-height reference on the physical
robot and inspect the resulting overlay.
