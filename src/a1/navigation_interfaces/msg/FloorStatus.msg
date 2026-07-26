# Current floor estimate from floor_mapping.
# The actual 2D map should be published separately as nav_msgs/OccupancyGrid.

std_msgs/Header header

int32 floor_id
string floor_name

# Height band for the active floor in the global/map frame, meters.
float32 z_min
float32 z_max

# 0.0 to 1.0 floor classification confidence.
float32 confidence

bool is_transition

uint8 TRANSITION_NONE=0
uint8 TRANSITION_STAIRS=1
uint8 TRANSITION_ELEVATOR=2
uint8 TRANSITION_RAMP=3
uint8 TRANSITION_UNKNOWN=255
uint8 transition_type

bool map_available

uint8 STATUS_UNKNOWN=0
uint8 STATUS_STABLE=1
uint8 STATUS_SWITCHING=2
uint8 STATUS_LOST=3
uint8 STATUS_ERROR=4
uint8 status

string message
