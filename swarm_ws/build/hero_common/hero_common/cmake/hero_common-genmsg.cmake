# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hero_common: 2 messages, 5 services")

set(MSG_I_FLAGS "-Ihero_common:/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hero_common_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg" NAME_WE)
add_custom_target(_hero_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hero_common" "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg" NAME_WE)
add_custom_target(_hero_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hero_common" "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg" ""
)

get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv" NAME_WE)
add_custom_target(_hero_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hero_common" "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv" ""
)

get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv" NAME_WE)
add_custom_target(_hero_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hero_common" "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv" ""
)

get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv" NAME_WE)
add_custom_target(_hero_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hero_common" "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv" ""
)

get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv" NAME_WE)
add_custom_target(_hero_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hero_common" "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv" ""
)

get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv" NAME_WE)
add_custom_target(_hero_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hero_common" "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
)
_generate_msg_cpp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
)

### Generating Services
_generate_srv_cpp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
)
_generate_srv_cpp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
)
_generate_srv_cpp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
)
_generate_srv_cpp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
)
_generate_srv_cpp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
)

### Generating Module File
_generate_module_cpp(hero_common
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hero_common_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hero_common_generate_messages hero_common_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_cpp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_cpp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_cpp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_cpp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_cpp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_cpp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_cpp _hero_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hero_common_gencpp)
add_dependencies(hero_common_gencpp hero_common_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hero_common_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
)
_generate_msg_eus(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
)

### Generating Services
_generate_srv_eus(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
)
_generate_srv_eus(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
)
_generate_srv_eus(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
)
_generate_srv_eus(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
)
_generate_srv_eus(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
)

### Generating Module File
_generate_module_eus(hero_common
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hero_common_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hero_common_generate_messages hero_common_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_eus _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_eus _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_eus _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_eus _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_eus _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_eus _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_eus _hero_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hero_common_geneus)
add_dependencies(hero_common_geneus hero_common_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hero_common_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
)
_generate_msg_lisp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
)

### Generating Services
_generate_srv_lisp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
)
_generate_srv_lisp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
)
_generate_srv_lisp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
)
_generate_srv_lisp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
)
_generate_srv_lisp(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
)

### Generating Module File
_generate_module_lisp(hero_common
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hero_common_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hero_common_generate_messages hero_common_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_lisp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_lisp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_lisp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_lisp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_lisp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_lisp _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_lisp _hero_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hero_common_genlisp)
add_dependencies(hero_common_genlisp hero_common_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hero_common_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
)
_generate_msg_nodejs(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
)

### Generating Services
_generate_srv_nodejs(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
)
_generate_srv_nodejs(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
)
_generate_srv_nodejs(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
)
_generate_srv_nodejs(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
)
_generate_srv_nodejs(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
)

### Generating Module File
_generate_module_nodejs(hero_common
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hero_common_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hero_common_generate_messages hero_common_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_nodejs _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_nodejs _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_nodejs _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_nodejs _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_nodejs _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_nodejs _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_nodejs _hero_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hero_common_gennodejs)
add_dependencies(hero_common_gennodejs hero_common_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hero_common_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
)
_generate_msg_py(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
)

### Generating Services
_generate_srv_py(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
)
_generate_srv_py(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
)
_generate_srv_py(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
)
_generate_srv_py(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
)
_generate_srv_py(hero_common
  "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
)

### Generating Module File
_generate_module_py(hero_common
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hero_common_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hero_common_generate_messages hero_common_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_py _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg" NAME_WE)
add_dependencies(hero_common_generate_messages_py _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_py _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_py _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_py _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_py _hero_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv" NAME_WE)
add_dependencies(hero_common_generate_messages_py _hero_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hero_common_genpy)
add_dependencies(hero_common_genpy hero_common_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hero_common_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hero_common
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(hero_common_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hero_common_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hero_common
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(hero_common_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hero_common_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hero_common
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(hero_common_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hero_common_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hero_common
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(hero_common_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hero_common_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hero_common
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(hero_common_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hero_common_generate_messages_py std_msgs_generate_messages_py)
endif()
