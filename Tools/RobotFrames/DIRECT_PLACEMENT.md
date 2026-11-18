# Direct pick and place

**Simple Pick and Place** opens the single-item task. In **Pick settings**,
choose **Center** and hold the bumper to capture the pickup contact, or choose
**Item surface** and follow the three-corner guide. Double-press the bumper to
capture the final placement contact. The separate TCP preview remains available.

**Repetitive Pick and Place** opens the batch setup. Enter the tile side,
thickness and number of tiles in the stack. The initial dimensions are
200 × 200 × 7 mm. Use either pickup method:

- **Pick centre · 1 point** records one controller-tip contact at the centre of the
  top tile. Use source yaw to align the green stack preview with the tiles.
- **Pick · 3 corners** opens the rotating tile guide. Touch three consecutive
  corners counterclockwise to fit both the centre and edge direction.

Choose **Place surface · 1 point** and touch the support surface under the
first tile's corner. The physical point marks the bottom of the tile.
**Pattern angle** sets the direction of the rows and columns. **Scan floor**
can also set the placement point on a Magic Leap 2 recognized floor.

Review the source stack, destination pattern and complete robot path before
executing. The robot moves each tile directly from the source to its final
location. It releases suction at final placement. Capture the new top tile
before planning another batch after execution.

## ROS contract

Build the matching `codex/direct-repetitive-placement` branch of
`UR16UnityControl` and restart the full ROS bringup to start a fresh MoveIt
planning scene. Unity checks the running node's read-only
`tile_floor_task_version=2` before publishing an updated
`ur16e_interfaces/TileFloorTask` on `/tile_floor_plan_target`. The schema is
in [TileFloorTask.msg](RosMessageSchemas/TileFloorTask.msg).

Source and destination are static, zero-stamp `ur_base_link` poses. The source
contains the centre pose or the three measured corners, identified by
`source_from_surface`. The destination pose carries the first tile's bottom
corner, with +X along columns, +Y along rows, and +Z upward. The wire format
also carries the tile dimensions. Zero stack pitch uses tile thickness.

The acknowledged plan ID must match the returned scene and ordered solution
before **Execute plan** becomes available. Unity sends that ID once to
`/execute_tile_floor_plan`. Review permission expires when the job, model,
alignment, or ROS connection changes.
