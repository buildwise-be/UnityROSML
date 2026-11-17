# Plan a row-major floor layout from a stack of known 200 x 200 x 7 mm tiles.
string request_id
uint32 timeout_seconds
# Three consecutive TOP corners of the topmost source tile in ur_base_link:
# P0(-L/2,-W/2), P1(+L/2,-W/2), P2(+L/2,+W/2). Local +Z points upward.
geometry_msgs/Point[3] source_top_corners
# Corner at the BOTTOM of the first floor tile. Static ur_base_link pose with
# zero timestamp. Local +X/+Y follow the pattern edges and +Z points upward.
geometry_msgs/PoseStamped floor_corner
uint32 tile_count
uint8 ROW_OFFSET_NONE=0
uint8 ROW_OFFSET_THIRD=1
uint8 ROW_OFFSET_HALF=2
uint8 row_offset
uint8 mortar_mm
