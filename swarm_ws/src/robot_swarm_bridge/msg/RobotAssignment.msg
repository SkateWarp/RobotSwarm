# Robot assignment for collaborative tasks
string robot_id            # Robot identifier
string role                # leader, follower, pusher_front, pusher_left, pusher_right, pusher_back
geometry_msgs/Pose target_pose  # Target pose relative to object or formation
float32 effort             # Force/effort to apply (0.0-1.0)
bool is_active             # True if robot is currently executing
