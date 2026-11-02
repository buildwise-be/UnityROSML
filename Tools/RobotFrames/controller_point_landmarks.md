# Controller contact landmarks for the UR16e

`RobotCoverLandmarks.CreateDefaults()` uses five blue cover centres, ordered
from base to tool: **Base cover, Shoulder, Elbow, Wrist 2, Wrist 3**.
Base cover is the lowest blue shoulder-pan cover on `ur_shoulder_link`,
not the gray stationary housing. Here Shoulder means the shoulder-lift
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

| Order | Contact | Link | Mesh | Surface node | X | Y | Z |
| --- | --- | --- | --- | --- | ---: | ---: | ---: |
| 1 | Base cover | `ur_shoulder_link` | `ur10e/visual/shoulder.dae` | `eSeries_UR10e_003` | -0.000003 | 0.000008 | 0.087942 |
| 2 | Shoulder | `ur_upper_arm_link` | `ur16e/visual/upperarm.dae` | `eSeries_UR10e_004` | -0.000001 | 0.087242 | -0.000297 |
| 3 | Elbow | `ur_upper_arm_link` | `ur16e/visual/upperarm.dae` | `eSeries_UR10e_005` | 0 | 0.065129 | 0.477982 |
| 4 | Wrist 2 | `ur_wrist_1_link` | `ur10e/visual/wrist1.dae` | `eSeries_UR10e_021` | -0.000001 | 0.135142 | -0.054024 |
| 5 | Wrist 3 | `ur_wrist_2_link` | `ur10e/visual/wrist2.dae` | `eSeries_UR10e_030` | 0.000001 | -0.053495 | 0.119459 |

The first point is the blue pan-joint cap above the fixed housing. See the
[blue base-cover derivation](base_blue_cover_registration_landmark.md) for its
CAD circle fit, exterior ray intersection and moving-link relationship.
The existing four covers follow it in their original relative order.

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
| `ur10e/visual/shoulder.dae` | `e8acdd7a52ccb4c11fbbf06b4a9fd5932b7b984bbd177c0e5fd53135501c4f23` |
| `ur16e/visual/upperarm.dae` | `cec3f8c5744f8ce28ccbf46384486fe006359b94cc40597adad91ff195035c8d` |
| `ur10e/visual/wrist1.dae` | `c00bc3196230d9a5bccdd24614122be2aa98c75f0402de6386ecaa8a851d2899` |
| `ur10e/visual/wrist2.dae` | `0c4c85a9902375aed81e5f6b147fe7c88ac4431861975639c0d512d479428279` |

## Derivation and verification

1. Select the specified visual-scene nodes bound to `URBlue-material`.
2. Read their COLLADA position arrays and apply each node's 4x4 matrix,
   including its millimetre-to-metre scale and upper-arm translation.
3. Project the blue mesh vertices into the plane perpendicular to the cover's
   axis. Fit a circle to the convex hull to estimate the circular centre.
   The hull radii are approximately 76.37, 76.36, 60.08, 47.00, and 47.01 mm.
4. Intersect the mesh triangles with a ray through that centre along the cover
   axis. Use the exterior surface intersection: positive mesh Z for Base cover,
   positive mesh Y for Shoulder
   and Elbow, negative mesh Z for Wrist 2, negative mesh Y for Wrist 3.
   This avoids screw protrusions and the centre of the cap's thickness.
5. Intersect at the final rounded radial centre coordinates, then round the
   resulting surface depth to the stored micrometre value. The centre region
   is slightly nonplanar, so reusing a depth intersected at a nominal centre
   instead of the fitted centre can shift the contact by tens of micrometres.

`RobotCoverLandmarkTests.DefaultContactIsOnTheActualMeshOuterSurface`
independently reads the actual DAE triangle/polylist data and repeats the
exterior-surface ray intersection for each stored point. It requires agreement
within **0.02 mm in the source mesh**. Additional tests check complete feedback,
wrong-mesh rejection, independence from tool choice, and correct cover movement
when a wrist joint changes.

The CAD surface check is not an accuracy claim about the physical robot or the
controller. The real plastic covers, where the user judges their centres, the
physical controller tip's offset from OpenXR's tracked pointer, and headset
tracking can each introduce errors. The contact offset should be checked on
the actual controller, and the five-cover identification and final overlay
should be verified on the robot. A low five-point residual alone cannot detect
a shared tip offset or a consistent error in all cover definitions.

## Registration feedback and final position correction

All five contacts are required before fitting and revealing the XR robot.
The helper marks the current contact on a geometry-only miniature, starting
with the blue base cover. It rotates around world vertical at 36 degrees per
second (one full turn in 10 seconds); this does not change the model landmarks
or captured joint snapshot.

A failed fit opens a dedicated recovery panel. It explains that no new placement
was applied and planning stays blocked, reports RMS/worst error against their
limits and the largest-mismatch contact, and offers Undo or Restart. Undo
recaptures point 5; an earlier suspect contact requires restarting the sequence.

After a successful fit, **Adjust XYZ** opens translation-only controls in the
fitted ROS `ur_base_link` axes. Steps are 0.1, 1 and 5 mm. Total displacement is
limited to a 20 mm radius from the original fit across adjustment sessions;
Reset returns to that fit. Done finishes the edit and invalidates previous
plans. The controls move the XR overlay only, retain live joint angles and
physical contact markers, and leave the original fit residuals unchanged.
Tracking resets invalidate this session-local correction together with the fit.
