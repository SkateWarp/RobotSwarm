# Install script for directory: /home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/anyelo/RobotSwarm/swarm_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hero_common/msg" TYPE FILE FILES
    "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Encoder.msg"
    "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/msg/Motor.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hero_common/srv" TYPE FILE FILES
    "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetPID.srv"
    "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetOdom.srv"
    "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetFrequency.srv"
    "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetMotor.srv"
    "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/srv/SetIRCalibration.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hero_common/cmake" TYPE FILE FILES "/home/anyelo/RobotSwarm/swarm_ws/build/hero_common/hero_common/catkin_generated/installspace/hero_common-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/anyelo/RobotSwarm/swarm_ws/devel/include/hero_common")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/anyelo/RobotSwarm/swarm_ws/devel/share/roseus/ros/hero_common")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/anyelo/RobotSwarm/swarm_ws/devel/share/common-lisp/ros/hero_common")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/anyelo/RobotSwarm/swarm_ws/devel/share/gennodejs/ros/hero_common")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/anyelo/RobotSwarm/swarm_ws/devel/lib/python3/dist-packages/hero_common")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/anyelo/RobotSwarm/swarm_ws/devel/lib/python3/dist-packages/hero_common")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/anyelo/RobotSwarm/swarm_ws/build/hero_common/hero_common/catkin_generated/installspace/hero_common.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hero_common/cmake" TYPE FILE FILES "/home/anyelo/RobotSwarm/swarm_ws/build/hero_common/hero_common/catkin_generated/installspace/hero_common-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hero_common/cmake" TYPE FILE FILES
    "/home/anyelo/RobotSwarm/swarm_ws/build/hero_common/hero_common/catkin_generated/installspace/hero_commonConfig.cmake"
    "/home/anyelo/RobotSwarm/swarm_ws/build/hero_common/hero_common/catkin_generated/installspace/hero_commonConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hero_common" TYPE FILE FILES "/home/anyelo/RobotSwarm/swarm_ws/src/hero_common/hero_common/package.xml")
endif()

