# Top-level mission_manager state.

std_msgs/Header header

uint8 INIT=0
uint8 WAIT_LOCALIZATION=1
uint8 WAIT_MAP=2
uint8 EXPLORING=3
uint8 APPROACHING_DANGER=4
uint8 CROSSING_FLOOR=5
uint8 RECOVERING=6
uint8 PAUSED=7
uint8 FINISHED=8
uint8 ERROR=9

uint8 state
int32 current_floor_id

string active_module
string active_goal
string message
