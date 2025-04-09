# TaskRequest.msg
uint32 task_id
string task_type
string task_params  # JSON format
uint32 robot_id  # 0 for auto-assignment
time timestamp

---