# Skill state
std_msgs/Header header
int64 skill_id
int64 meta_skill_id
int64 skill_type
int64 meta_skill_type
string skill_description
float64 time_since_skill_started
uint8 SKILL_STATUS_TO_START=0
uint8 SKILL_STATUS_RUNNING=1
uint8 SKILL_STATUS_FINISHED=2
uint8 SKILL_STATUS_PREEMPTED=3
uint8 skill_status
bool is_fresh