# A perceived wall opening.  A doorway is a geometric result; it is not an
# authorization to invoke a building-control service.

std_msgs/Header header

# Stable only within one localization_generation/floor_session_id pair.
uint32 detection_id
int32 localization_generation
uint64 floor_session_id

uint8 STRUCTURE_UNKNOWN=0
uint8 WALL_OPENING=1
uint8 DOORWAY=2
uint8 structure_type

uint8 STATE_UNKNOWN=0
uint8 OPEN=1
uint8 CLOSED=2
uint8 PARTIALLY_OPEN=3
uint8 BLOCKED=4
uint8 state

geometry_msgs/Point center
geometry_msgs/Vector3 normal
geometry_msgs/Point left_boundary
geometry_msgs/Point right_boundary
float32 width
float32 usable_width

# The two safe observation/approach poses are on opposite sides of the wall.
geometry_msgs/Pose entry_pose
geometry_msgs/Pose exit_pose
bool passage_direction_resolved

bool traversable
float32 confidence
uint32 observation_count
bool stable

# Empty unless a separate, reliable association process has matched a public
# building control ID. floor_mapping itself never derives this from geometry.
bool control_id_matched
string control_door_id
float32 control_id_confidence
string door_kind
