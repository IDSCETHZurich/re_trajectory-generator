# Install script for directory: /home/demo/ros/stacks/demoLocal/kuka_IK

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
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK/libkuka_IK-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK/libkuka_IK-gnulinux.so"
         RPATH "/usr/local/lib:/usr/local/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/opt/ros/diamondback/stacks/orocos_toolchain_ros/rtt/install/lib")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK/libkuka_IK-gnulinux.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK" TYPE SHARED_LIBRARY FILES "/home/demo/ros/stacks/demoLocal/kuka_IK/lib/orocos/gnulinux/libkuka_IK-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK/libkuka_IK-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK/libkuka_IK-gnulinux.so"
         OLD_RPATH "/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/demoLocal/kuka_IK/build:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/opt/ros/diamondback/stacks/orocos_toolchain_ros/rtt/install/lib:"
         NEW_RPATH "/usr/local/lib:/usr/local/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/opt/ros/diamondback/stacks/orocos_toolchain_ros/rtt/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK/libkuka_IK-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/kuka_IK/libkuka_IK-gnulinux.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkuka_IK-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkuka_IK-gnulinux.so"
         RPATH "/usr/local/lib:/usr/local/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/opt/ros/diamondback/stacks/orocos_toolchain_ros/rtt/install/lib")
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkuka_IK-gnulinux.so")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/demo/ros/stacks/demoLocal/kuka_IK/lib/orocos/gnulinux/libkuka_IK-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkuka_IK-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkuka_IK-gnulinux.so"
         OLD_RPATH "/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/home/demo/ros/stacks/demoLocal/kuka_IK/build:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/opt/ros/diamondback/stacks/orocos_toolchain_ros/rtt/install/lib:"
         NEW_RPATH "/usr/local/lib:/usr/local/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/log4cpp/install/lib:/home/demo/ros/stacks/orocos_kinematics_dynamics/orocos_kdl/install/lib:/opt/ros/diamondback/stacks/common_msgs/sensor_msgs/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosbag/lib:/opt/ros/diamondback/stacks/ros_comm/tools/topic_tools/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp/lib:/opt/ros/diamondback/stacks/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/diamondback/stacks/ros_comm/tools/rosconsole/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/rostime/lib:/opt/ros/diamondback/stacks/ros_comm/utilities/cpp_common/lib:/opt/ros/diamondback/ros/core/roslib/lib:/opt/ros/diamondback/ros/tools/rospack/lib:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/plugins:/opt/ros/diamondback/stacks/orocos_toolchain_ros/ocl/install/lib/orocos/gnulinux/ocl/types:/opt/ros/diamondback/stacks/orocos_toolchain_ros/rtt/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkuka_IK-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libkuka_IK-gnulinux.so")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/kuka_IK" TYPE FILE FILES "/home/demo/ros/stacks/demoLocal/kuka_IK/src/kuka_IK-component.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/demo/ros/stacks/demoLocal/kuka_IK/build/kuka_ik-gnulinux.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/demo/ros/stacks/demoLocal/kuka_IK/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/demo/ros/stacks/demoLocal/kuka_IK/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
