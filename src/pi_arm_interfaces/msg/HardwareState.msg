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
# Pack-reference calibration and actuator-coordinate diagnostics.
bool calibrated
bool kinematics_valid
bool control_permitted
string calibration_id
float64[] raw_protocol_positions_deg
float64[] raw_output_positions_rad
float64[] software_output_offsets_rad
float64[] model_output_positions_rad
string kinematics_error_code
int32 kinematics_error_axis
float64 kinematics_error_value
string kinematics_error_message
string diagnostic
# Dedicated direction-verification mode, independent of normal motion permission.
bool commissioning
string can_interface
bool motor_test_permitted
bool joint_test_permitted
bool axis_test_active
uint64 axis_test_id
string axis_test_status
float64[] protocol_angle_ratios
int32[] motor_directions
