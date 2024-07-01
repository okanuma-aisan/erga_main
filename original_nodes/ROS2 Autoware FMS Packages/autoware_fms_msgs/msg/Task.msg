# task type
uint8 MOVE = 0

# status
uint8 TODO = 0
uint8 DOING = 1
uint8 DONE = 2
uint8 ABORTING = 3
uint8 ABORTED = 4
uint8 DISABLED = 5

string task_id
uint8 task_type
bool is_reserved
uint8 status
autoware_fms_msgs/Place origin
autoware_fms_msgs/Place destination
builtin_interfaces/Time plan_start_time
builtin_interfaces/Time plan_end_time
int32 duration_sec
string[] route_ids
builtin_interfaces/Time actual_start_time
