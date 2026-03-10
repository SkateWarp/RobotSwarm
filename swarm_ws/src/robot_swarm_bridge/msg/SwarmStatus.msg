# Overall swarm status message
Header header
string task_type            # follow_leader, formation, transport, idle
string status               # initializing, running, paused, completed, failed
uint8 robot_count          # Total robots configured
uint8 active_robots        # Number of active robots
float32 progress           # Task progress 0.0-1.0
RobotStatus[] robot_statuses  # Individual robot statuses
