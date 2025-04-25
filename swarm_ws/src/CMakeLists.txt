cmake_minimum_required(VERSION 3.0.2)
project(robot_swarm_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  gazebo_msgs
  message_generation
  tf
  visualization_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotStatus.msg
  SensorData.msg
  TaskRequest.msg
  TaskProgress.msg
  DrawingCommand.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SpawnRobot.srv
  DeleteRobot.srv
  AssignTask.srv
  CoordinateTask.srv
)

## Generate added messages and services with any dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    std_srvs
    geometry_msgs 
    nav_msgs 
    sensor_msgs 
    gazebo_msgs 
    message_runtime 
    tf
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/robot_manager.py
  scripts/task_manager.py
  scripts/turtlebot_font_drawer.py
  scripts/initial_robot_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and config files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE
)