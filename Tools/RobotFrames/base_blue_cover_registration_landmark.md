# First registration contact: lowest blue base cover

The registration order is **Base cover, Shoulder, Elbow, Wrist 2, Wrist 3**.
"Base cover" means the first blue circular cover above the stationary base
housing, at the shoulder-pan joint. Its mesh belongs to the moving
`ur_shoulder_link`, not `ur_base_link` or `ur_base_link_inertia`. The gray fixed
housing is not a registration target.

The first contact uses visual 0 of `ur_shoulder_link`, with mesh suffix
`/ur10e/visual/shoulder.dae`. The UR16e model intentionally shares this upstream
UR10e geometry. The source file is:

`Assets/Prefabs/ur16e_kenos/meshes/ur_description/meshes/ur10e/visual/shoulder.dae`

SHA-256: `e8acdd7a52ccb4c11fbbf06b4a9fd5932b7b984bbd177c0e5fd53135501c4f23`.

The identified node is `eSeries_UR10e_003`, bound to `URBlue-material`. Its node
matrix converts the COLLADA positions to metre coordinates in the visual frame.
A least-squares circle fit to the 162-vertex convex hull in mesh X/Y gives:

```text
circle centre XY (m): (-0.000003284110, 0.000007729725)
circle radius (m):     0.076372156436
```

After rounding X/Y to micrometres, a ray along positive mesh Z through that
centre intersects the actual blue surface at:

```text
exterior contact (m): (-0.000003, 0.000008, 0.087941613310)
stored contact (m):   (-0.000003, 0.000008, 0.087942)
guide outward normal: (0,        0,        1)
```

The stored point differs from the exterior CAD face by approximately 0.00039 mm.
This is a surface contact rather than the joint origin or the cap bounding-box
centre. The runtime composes the active description's calibrated shoulder-pan
origin, live FK and visual origin; it does not assume a fixed-base position.
The cap centre is nearly on the pan axis, so a pan rotation changes its position
only slightly while the downstream arm moves substantially.

`RobotCoverLandmarkTests.DefaultContactIsOnTheActualMeshOuterSurface` independently
reads the node matrix, positions and triangle/polylist indices and requires the
stored point to match the positive-Z exterior intersection within 0.02 mm.
Additional tests verify the five contacts' base-to-tool order, the pan link and
calibrated origin, wrist motion and tool independence, and preservation of all
five point pairs in the rigid fit and its residuals.

`RobotCoverLandmarks.UpgradeDefaults` migrates only exact untouched legacy
four-cover defaults or the superseded four-covers-plus-gray-base definition.
Any changed coordinate, link, mesh or label preserves the custom array instead
of silently replacing a measured setup.

CAD agreement does not measure the physical cover, controller-tip offset or
headset tracking. Verify the blue-cap identification and resulting overlay on
the real robot.
