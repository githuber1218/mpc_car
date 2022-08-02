# Install script for directory: /home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wang/xiaoche/yunjing/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs/msg" TYPE FILE FILES
    "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/msg/TracerMotorState.msg"
    "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/msg/UartTracerMotorState.msg"
    "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/msg/TracerLightState.msg"
    "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/msg/TracerLightCmd.msg"
    "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/msg/TracerStatus.msg"
    "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/msg/UartTracerStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs/cmake" TYPE FILE FILES "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/wang/xiaoche/yunjing/devel/include/tracer_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/wang/xiaoche/yunjing/devel/share/roseus/ros/tracer_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/wang/xiaoche/yunjing/devel/share/common-lisp/ros/tracer_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/wang/xiaoche/yunjing/devel/share/gennodejs/ros/tracer_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/wang/xiaoche/yunjing/devel/lib/python3/dist-packages/tracer_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/wang/xiaoche/yunjing/devel/lib/python3/dist-packages/tracer_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs/cmake" TYPE FILE FILES "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs/cmake" TYPE FILE FILES
    "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgsConfig.cmake"
    "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs" TYPE FILE FILES "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs/cmake" TYPE FILE FILES "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs/cmake" TYPE FILE FILES
    "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgsConfig.cmake"
    "/home/wang/xiaoche/yunjing/build/tracer_ros/tracer_msgs/catkin_generated/installspace/tracer_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tracer_msgs" TYPE FILE FILES "/home/wang/xiaoche/yunjing/src/tracer_ros/tracer_msgs/package.xml")
endif()

