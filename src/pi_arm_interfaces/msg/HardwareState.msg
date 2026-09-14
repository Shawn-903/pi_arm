builtin_interfaces/Time stamp
bool connected
string driver_mode
string management_operation
string[] joint_names
float64[] positions
float64[] velocities
# Joint-level availability aggregated through the actuator dependency map.
# These arrays are indexed by joint_names and are not independent motor state.
bool[] enabled
bool[] errors
bool[] moving
bool[] fresh
int32[] error_codes
float64[] ages_sec
# Raw controller state indexed by motor_ids. Consumers must use motor_ids
# rather than assuming array indexes.
int32[] motor_ids
bool[] motor_enabled
bool[] motor_errors
bool[] motor_moving
bool[] motor_fresh
int32[] motor_error_codes
float64[] motor_ages_sec
string diagnostic
