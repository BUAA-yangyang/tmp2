# Clustered and de-duplicated danger source maintained by result_manager.

std_msgs/Header header

uint32 source_id

# Cluster center in header.frame_id, meters.
geometry_msgs/Point position

# Fused confidence, 0.0 to 1.0.
float32 confidence

# Number of accepted observations merged into this source.
uint32 observation_count

string class_name

# Examples: tentative, confirmed, rejected, stale.
string status
