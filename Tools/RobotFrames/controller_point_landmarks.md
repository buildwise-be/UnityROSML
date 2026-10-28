# Controller contact landmarks for the UR16e

`RobotCoverLandmarks.CreateDefaults()` uses the four blue rear-cover centres
visible in the supplied robot photograph, ordered from the base toward the tool:
**Shoulder, Elbow, Wrist 2, Wrist 3**. Here Shoulder means the shoulder-lift
joint cover. The wrist-1 cover is on the opposite side of the forearm and is
omitted. The last two labels identify the covered joints; their meshes belong
to the preceding links.

These are **surface contacts**, not joint-axis origins. Both large covers are
part of the upper-arm visual mesh. Using the elbow joint origin or a bounding
box centre would place the tip inside the robot and introduce a systematic
alignment error.

## Default coordinates and source geometry

Positions below are metres in the ROS FLU axes of the visual mesh, before the
model's visual origin, visual scale, and link forward kinematics are applied.
`pointInVisualRos` is exposed for measurement-based corrections in the inspector.

| Order | Cover | Link | Mesh | Blue node | X | Y | Z |
| --- | --- | --- | --- | --- | ---: | ---: | ---: |
| 1 | Shoulder | `ur_upper_arm_link` | `ur16e/visual/upperarm.dae` | `eSeries_UR10e_004` | -0.000001 | 0.087242 | -0.000297 |
| 2 | Elbow | `ur_upper_arm_link` | `ur16e/visual/upperarm.dae` | `eSeries_UR10e_005` | 0 | 0.065129 | 0.477982 |
| 3 | Wrist 2 | `ur_wrist_1_link` | `ur10e/visual/wrist1.dae` | `eSeries_UR10e_021` | -0.000001 | 0.135142 | -0.054024 |
| 4 | Wrist 3 | `ur_wrist_2_link` | `ur10e/visual/wrist2.dae` | `eSeries_UR10e_030` | 0.000001 | -0.053495 | 0.119459 |

The UR16e model intentionally uses the upstream UR10e wrist geometry. The
runtime helper checks each expected mesh suffix to prevent silently applying
these points to a different arm mesh. It composes the active ROS description's
visual origin and calibrated link FK, so these values do not duplicate nominal
joint offsets or assume the zero pose. All six revolute joints must be present
and finite in the supplied feedback map. Tool changes leave these arm points
unchanged.

Source files are under
`Assets/Prefabs/ur16e_kenos/meshes/ur_description/meshes/`:

| File | SHA-256 |
| --- | --- |
| `ur16e/visual/upperarm.dae` | `cec3f8c5744f8ce28ccbf46384486fe006359b94cc40597adad91ff195035c8d` |
| `ur10e/visual/wrist1.dae` | `c00bc3196230d9a5bccdd24614122be2aa98c75f0402de6386ecaa8a851d2899` |
| `ur10e/visual/wrist2.dae` | `0c4c85a9902375aed81e5f6b147fe7c88ac4431861975639c0d512d479428279` |

## Derivation and verification

1. Select the specified visual-scene nodes bound to `URBlue-material`.
2. Read their COLLADA position arrays and apply each node's 4x4 matrix,
   including its millimetre-to-metre scale and upper-arm translation.
3. Project the blue mesh vertices into the plane perpendicular to the cover's
   axis. Fit a circle to the convex hull to estimate the circular centre.
   The hull radii are approximately 76.36, 60.08, 47.00, and 47.01 mm.
4. Intersect the mesh triangles with a ray through that centre along the cover
   axis. Use the exterior surface intersection: positive mesh Y for Shoulder
   and Elbow, negative mesh Z for Wrist 2, negative mesh Y for Wrist 3.
   This avoids screw protrusions and the centre of the cap's thickness.
5. Intersect at the final rounded radial centre coordinates, then round the
   resulting surface depth to the stored micrometre value. The centre region
   is slightly nonplanar, so reusing a depth intersected at a nominal centre
   instead of the fitted centre can shift the contact by tens of micrometres.

`RobotCoverLandmarkTests.DefaultContactIsOnTheActualBlueMeshOuterSurface`
independently reads the actual DAE triangle/polylist data and repeats the
exterior-surface ray intersection for each stored point. It requires agreement
within **0.02 mm in the source mesh**. Additional tests check complete feedback,
wrong-mesh rejection, independence from tool choice, and correct cover movement
when a wrist joint changes.

The CAD surface check is not an accuracy claim about the physical robot or the
controller. The real plastic covers, where the user judges their centres, the
physical controller tip's offset from OpenXR's tracked pointer, and headset
tracking can each introduce errors. The contact offset should be checked on
the actual controller, and the four-cover identification and final overlay
should be verified on the robot. A low four-point residual alone cannot detect
a shared tip offset or a consistent error in all cover definitions.
