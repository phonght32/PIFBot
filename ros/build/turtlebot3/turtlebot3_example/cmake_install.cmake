# Install script for directory: /home/phong/Documents/thesis/ros/src/turtlebot3/turtlebot3_example

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/phong/Documents/thesis/ros/install")
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
  include("/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_example/action" TYPE FILE FILES "/home/phong/Documents/thesis/ros/src/turtlebot3/turtlebot3_example/action/Turtlebot3.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_example/msg" TYPE FILE FILES
    "/home/phong/Documents/thesis/ros/devel/share/turtlebot3_example/msg/Turtlebot3Action.msg"
    "/home/phong/Documents/thesis/ros/devel/share/turtlebot3_example/msg/Turtlebot3ActionGoal.msg"
    "/home/phong/Documents/thesis/ros/devel/share/turtlebot3_example/msg/Turtlebot3ActionResult.msg"
    "/home/phong/Documents/thesis/ros/devel/share/turtlebot3_example/msg/Turtlebot3ActionFeedback.msg"
    "/home/phong/Documents/thesis/ros/devel/share/turtlebot3_example/msg/Turtlebot3Goal.msg"
    "/home/phong/Documents/thesis/ros/devel/share/turtlebot3_example/msg/Turtlebot3Result.msg"
    "/home/phong/Documents/thesis/ros/devel/share/turtlebot3_example/msg/Turtlebot3Feedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_example/cmake" TYPE FILE FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_example-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/phong/Documents/thesis/ros/devel/include/turtlebot3_example")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/phong/Documents/thesis/ros/devel/share/roseus/ros/turtlebot3_example")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/phong/Documents/thesis/ros/devel/share/common-lisp/ros/turtlebot3_example")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/phong/Documents/thesis/ros/devel/share/gennodejs/ros/turtlebot3_example")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/phong/Documents/thesis/ros/devel/lib/python2.7/dist-packages/turtlebot3_example")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/phong/Documents/thesis/ros/devel/lib/python2.7/dist-packages/turtlebot3_example" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/phong/Documents/thesis/ros/devel/lib/python2.7/dist-packages/turtlebot3_example" FILES_MATCHING REGEX "/home/phong/Documents/thesis/ros/devel/lib/python2.7/dist-packages/turtlebot3_example/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_example.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_example/cmake" TYPE FILE FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_example-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_example/cmake" TYPE FILE FILES
    "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_exampleConfig.cmake"
    "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_exampleConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_example" TYPE FILE FILES "/home/phong/Documents/thesis/ros/src/turtlebot3/turtlebot3_example/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_obstacle")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_client")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_pointop_key")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_bumper")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_cliff")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_sonar")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_illumination")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_example" TYPE PROGRAM FILES "/home/phong/Documents/thesis/ros/build/turtlebot3/turtlebot3_example/catkin_generated/installspace/turtlebot3_marker_server")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_example" TYPE DIRECTORY FILES
    "/home/phong/Documents/thesis/ros/src/turtlebot3/turtlebot3_example/launch"
    "/home/phong/Documents/thesis/ros/src/turtlebot3/turtlebot3_example/rviz"
    )
endif()

