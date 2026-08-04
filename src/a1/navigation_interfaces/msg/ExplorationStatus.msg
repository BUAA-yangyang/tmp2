# High-level exploration state. Goals should still be geometry_msgs/PoseStamped
# or ExploreFloor.action goals, not custom navigation messages.

std_msgs/Header header

uint8 IDLE=0
uint8 SELECTING_TARGET=1
uint8 NAVIGATING=2
uint8 COMPLETED=3
uint8 FAILED=4
uint8 PAUSED=5
# Values 0..5 are retained. New detailed lifecycle states are append-only.
uint8 RECORD_START=6
uint8 UPDATE_COVERAGE=7
uint8 EXPLORATION_DONE=8
uint8 RETURNING=9
uint8 RETURNED=10
uint8 CANCELLED=11
uint8 REQUEST_ENTRY_DOOR_OPEN=12
uint8 TRANSIT_TO_ENTRY=13
uint8 ENTERED_FLOOR=14

uint8 state
int32 floor_id

# 0.0 to 1.0 estimated explored traversable area.
float32 coverage_ratio

geometry_msgs/PoseStamped current_target
string message
