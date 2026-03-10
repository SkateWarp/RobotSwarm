# Collaborative transport task definition
Header header
string object_id           # Unique object identifier
geometry_msgs/Pose object_pose  # Current object position
geometry_msgs/Pose target_pose  # Delivery destination
float32 object_mass        # Object mass in kg
string[] assigned_robots   # Robot IDs assigned to this task
float32 transport_progress # 0.0-1.0 progress to destination
string status              # approaching, gripping, transporting, completed
