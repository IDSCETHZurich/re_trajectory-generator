# Install script for directory: /home/demo/ros/stacks/demoLocal/trajectory_generators

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtrajectory_generators-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtrajectory_generators-gnulinux.so"
         RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/trajectory_generators:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/lib:/home/demo/ros/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/geometry/tf_conversions/lib:/opt/ros/diamondback/stacks/geometry/tf/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/geometry/bullet/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/diamondback/stacks/geometry/kdl/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/home/demo/ros/stacks/orocos_toolchain_ros/rtt/install/lib")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtrajectory_generators-gnulinux.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/demo/ros/stacks/demoLocal/trajectory_generators/lib/orocos/gnulinux/libtrajectory_generators-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtrajectory_generators-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtrajectory_generators-gnulinux.so"
         OLD_RPATH "/home/demo/ros/stacks/orocos_toolchain_ros/ocl/lib:/home/demo/ros/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/geometry/tf_conversions/lib:/opt/ros/diamondback/stacks/geometry/tf/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/geometry/bullet/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/diamondback/stacks/geometry/kdl/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/demoLocal/trajectory_generators:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/home/demo/ros/stacks/orocos_toolchain_ros/rtt/install/lib:::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/trajectory_generators:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/lib:/home/demo/ros/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/geometry/tf_conversions/lib:/opt/ros/diamondback/stacks/geometry/tf/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/geometry/bullet/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/diamondback/stacks/geometry/kdl/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/home/demo/ros/stacks/orocos_toolchain_ros/rtt/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtrajectory_generators-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libtrajectory_generators-gnulinux.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators/libtrajectory_generators-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators/libtrajectory_generators-gnulinux.so"
         RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/trajectory_generators:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/lib:/home/demo/ros/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/geometry/tf_conversions/lib:/opt/ros/diamondback/stacks/geometry/tf/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/geometry/bullet/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/diamondback/stacks/geometry/kdl/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/home/demo/ros/stacks/orocos_toolchain_ros/rtt/install/lib")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators/libtrajectory_generators-gnulinux.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators" TYPE SHARED_LIBRARY FILES "/home/demo/ros/stacks/demoLocal/trajectory_generators/lib/orocos/gnulinux/libtrajectory_generators-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators/libtrajectory_generators-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators/libtrajectory_generators-gnulinux.so"
         OLD_RPATH "/home/demo/ros/stacks/orocos_toolchain_ros/ocl/lib:/home/demo/ros/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/geometry/tf_conversions/lib:/opt/ros/diamondback/stacks/geometry/tf/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/geometry/bullet/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/diamondback/stacks/geometry/kdl/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/demoLocal/trajectory_generators:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/home/demo/ros/stacks/orocos_toolchain_ros/rtt/install/lib:::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib/orocos/gnulinux:/usr/local/lib:/usr/local/lib/orocos/gnulinux/trajectory_generators:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/lib:/home/demo/ros/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/geometry/tf_conversions/lib:/opt/ros/diamondback/stacks/geometry/tf/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/geometry/bullet/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/message_filters/lib:/opt/ros/diamondback/stacks/geometry/kdl/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/home/demo/ros/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/home/demo/ros/stacks/orocos_toolchain_ros/rtt/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators/libtrajectory_generators-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/trajectory_generators/libtrajectory_generators-gnulinux.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/demo/ros/stacks/demoLocal/trajectory_generators/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/demo/ros/stacks/demoLocal/trajectory_generators/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
