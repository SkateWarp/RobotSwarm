# Formation shape definition
Header header
string formation_type       # line, triangle, circle, square, v, diamond
geometry_msgs/Point centroid  # Formation center point
geometry_msgs/Point[] robot_positions  # Target position for each robot
float32 spacing            # Distance between robots
bool is_moving             # True if formation is moving, false if static
geometry_msgs/Pose target_pose  # Target pose for moving formation
