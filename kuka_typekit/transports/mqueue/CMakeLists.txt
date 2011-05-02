# Generated from orogen/lib/orogen/templates/typekit/mqueue/CMakeLists.txt

find_package(Orocos-RTT)

# Set up generation of IDL files for MQueue support
IF (NOT OROCOS-RTT_MQUEUE_FOUND)
    MESSAGE(FATAL_ERROR "could not find development files for Orocos/MQueue")
ENDIF(NOT OROCOS-RTT_MQUEUE_FOUND)

set(MQueue_FILES "")

# Generate the MQueue transport
set(libname_mqueue kuka_typekit-transport-mqueue-${OROCOS_TARGET})
add_library(${libname_mqueue} SHARED
    /home/demo/ros/stacks/demoLocal/kuka_typekit/transports/mqueue/TransportPlugin.cpp
    /home/demo/ros/stacks/demoLocal/kuka_typekit/transports/mqueue/tFriCmdData_tFriHeader.cpp
    /home/demo/ros/stacks/demoLocal/kuka_typekit/transports/mqueue/tFriIntfState_tFriIntfStatistics.cpp
    /home/demo/ros/stacks/demoLocal/kuka_typekit/transports/mqueue/tFriKrlData_tFriMsrData.cpp
    /home/demo/ros/stacks/demoLocal/kuka_typekit/transports/mqueue/tFriRobotCommand_tFriRobotData.cpp
    /home/demo/ros/stacks/demoLocal/kuka_typekit/transports/mqueue/tFriRobotState.cpp
    ${MQueue_FILES})
target_link_libraries(${libname_mqueue}
    kuka_typekit-typekit-${OROCOS_TARGET}
    ${OROCOS-RTT_MQUEUE_LIBRARIES})

target_link_libraries(${libname_mqueue} ${RTT_TRANSPORT_MQUEUE_LIBRARIES})
target_link_libraries(${libname_mqueue} LINK_INTERFACE_LIBRARIES ${OROCOS-RTT_MQUEUE_LIBRARIES})

SET(PKG_CONFIG_FILE_MQueue ${CMAKE_CURRENT_BINARY_DIR}/kuka_typekit-transport-mqueue-${OROCOS_TARGET}.pc)
CONFIGURE_FILE(kuka_typekit-transport-mqueue.pc.in ${PKG_CONFIG_FILE_MQueue} @ONLY)

install(TARGETS ${libname_mqueue} LIBRARY DESTINATION lib/orocos/types)
install(FILES ${PKG_CONFIG_FILE_MQueue} DESTINATION lib/pkgconfig)
install(FILES
    /home/demo/ros/stacks/demoLocal/kuka_typekit/transports/mqueue/TransportPlugin.hpp
    DESTINATION include/orocos/kuka_typekit/transports/mqueue)

