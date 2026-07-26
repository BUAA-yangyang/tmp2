# Status of the only upper-layer module allowed to publish final /cmd_vel.
# Velocity inputs and output remain standard geometry_msgs/Twist topics.

std_msgs/Header header

uint8 SOURCE_NONE=0
uint8 SOURCE_NAVIGATION=1
uint8 SOURCE_BEHAVIOR=2
uint8 SOURCE_TELEOP=3
uint8 SOURCE_ESTOP=4

uint8 active_source

bool emergency_stop
bool output_enabled

geometry_msgs/Twist last_cmd

# Age of the active input command in seconds.
float32 active_source_age_s

string status
