cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
add_definitions(-D_UNIX_)

include_directories(/opt/pleora/ebus_sdk/include)
LINK_DIRECTORIES(/opt/pleora/ebus_sdk/lib)
LINK_DIRECTORIES(/opt/pleora/ebus_sdk/lib/genicam/bin/Linux64_x64)

rosbuild_add_executable(seneka_sony_camera src/sony_camera_node.cpp)


set(EBUS_LIB
        PvBase
        PvDevice
        PvBuffer
        PvGUIUtils
        PvGUI
        PvPersistence
        PvGenICam
        PvStreamRaw
        PvStream
        PvTransmitterRaw
        PvVirtualDevice
        EbNetworkLib
        PtUtilsLib
        PtConvertersLib
        PvSerial
        EbTransportLayerLib
)

set(GENICAM_LIB
        GCBase_gcc40_v2_2
        GenApi_gcc40_v2_2
        log4cpp_gcc40_v2_2
        Log_gcc40_v2_2
        MathParser_gcc40_v2_2
)



target_link_libraries(seneka_sony_camera  ${EBUS_LIB} ${GENICAM_LIB})
