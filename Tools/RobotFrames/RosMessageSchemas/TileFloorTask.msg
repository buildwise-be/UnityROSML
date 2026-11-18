# Plan a direct row-major floor layout from a measured stack of square tiles.
string request_id
uint32 timeout_seconds
float64 tile_side
float64 tile_thickness
# Top-to-top distance in metres. Zero uses tile_thickness.
float64 stack_pitch
bool source_from_surface
# Used for centre capture. Static ur_base_link pose on the top face.
geometry_msgs/PoseStamped source_top
# Three consecutive TOP corners of the topmost source tile in ur_base_link:
# P0(-L/2,-W/2), P1(+L/2,-W/2), P2(+L/2,+W/2). Local +Z points upward.
# Used when source_from_surface is true.
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
