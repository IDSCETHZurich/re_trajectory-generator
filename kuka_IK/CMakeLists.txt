#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.4.6)

rosbuild_init()

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
	rosbuild_init()
	#set the default path for built executables to the "bin" directory
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
	#set the default path for built libraries to the "lib" directory
	set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
	rosbuild_find_ros_package( rtt )
	find_package(Orocos-RTT HINTS ${rtt_PACKAGE_PATH}/install )
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
orocos_component(${PROJECT_NAME} src/KukaLWR_Kinematics.cpp src/kuka_IK-component.cpp)