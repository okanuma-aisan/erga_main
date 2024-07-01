# schedule_type
uint8 ONEWAY = 0
uint8 CYCLIC = 1
uint8 MAINTENANCE = 2

# status
uint8 DOING = 0
uint8 CANCELLING = 1
uint8 ABORTING = 2

string schedule_id
string project_id
string environment_id
uint8 schedule_type
bool is_reserved
string vehicle_id
uint8 status
builtin_interfaces/Time plan_start_time
int32 delta_sec
autoware_fms_msgs/Task[] tasks
builtin_interfaces/Time created_at
builtin_interfaces/Time updated_at
builtin_interfaces/Time plan_end_time
builtin_interfaces/Time actual_start_time
int32 duration_sec
autoware_fms_msgs/Tag[] tags
