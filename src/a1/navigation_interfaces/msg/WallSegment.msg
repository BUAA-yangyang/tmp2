# Stable planar wall segment observed by floor_mapping.
# The geometry is expressed in header.frame_id (normally odom).

std_msgs/Header header

# Stable only within one localization_generation/floor_session_id pair.
uint32 detection_id
int32 localization_generation
uint64 floor_session_id

geometry_msgs/Point start
geometry_msgs/Point end
geometry_msgs/Vector3 normal

float32 length
float32 fit_error
float32 height_support
float32 confidence
uint32 observation_count
bool stable

# Examples: observed, temporarily_lost.
string status
