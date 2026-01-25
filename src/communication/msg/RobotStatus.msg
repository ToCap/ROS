uint16 protocol_version

uint8 MODE_UNKNOWN=0
uint8 MODE_MANUAL=1
uint8 MODE_AUTO=2
uint8 MODE_ERROR=3
uint8 mode

bool is_armed
bool emergency_stop

uint8 NAV_IDLE=0
uint8 NAV_PLANNING=1
uint8 NAV_MOVING=2
uint8 NAV_AVOIDING=3
uint8 NAV_REACHED=4
uint8 NAV_FAILED=5
uint8 nav_state

float32 nav_progress
uint16 nav_error_code

uint16 error_code
string error_message

builtin_interfaces/Time stamp
