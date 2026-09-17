uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCEEDED=2
uint8 CANCELED=3
uint8 FAILED=4

uint64 task_id
string command
uint8 status
float32 progress
int32 code
string message

# Arrival verification. Position/error units are radians; errors = target - feedback.
# Errors are the latest verification sample and are retained at task completion.
string[] joint_names
float64[] target_positions
float64[] position_errors
float64 goal_position_tolerance
string arrival_state
