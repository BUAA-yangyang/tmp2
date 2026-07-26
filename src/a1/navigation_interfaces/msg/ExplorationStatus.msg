# High-level exploration state. Goals should still be geometry_msgs/PoseStamped
# or ExploreFloor.action goals, not custom navigation messages.

std_msgs/Header header

uint8 IDLE=0
uint8 SELECTING_TARGET=1
uint8 NAVIGATING=2
uint8 COMPLETED=3
uint8 FAILED=4
uint8 PAUSED=5

uint8 state
int32 floor_id

# 0.0 to 1.0 estimated explored traversable area.
float32 coverage_ratio

geometry_msgs/PoseStamped current_target
string message
