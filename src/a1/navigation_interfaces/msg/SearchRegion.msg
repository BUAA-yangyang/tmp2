# Optional semantic search region from floor_mapping.
# If exploration only needs geometry, use nav_msgs/OccupancyGrid directly.

std_msgs/Header header

int32 floor_id
string region_id

uint8 TYPE_UNKNOWN=0
uint8 TYPE_ROOM=1
uint8 TYPE_CORRIDOR=2
uint8 TYPE_DOORWAY=3
uint8 TYPE_ELEVATOR=4
uint8 TYPE_STAIRS=5
uint8 region_type

# Boundary in header.frame_id, meters.
geometry_msgs/Polygon boundary

float32 priority
bool is_accessible
bool is_visited

uint8 STATUS_UNKNOWN=0
uint8 STATUS_OPEN=1
uint8 STATUS_VISITED=2
uint8 STATUS_BLOCKED=3
uint8 STATUS_UNREACHABLE=4
uint8 status

string message
