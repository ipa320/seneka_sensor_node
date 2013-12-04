cmake_minimum_required(VERSION 2.8.3)
project(seneka_sony_camera)

# the following statement is to ensure that the required SDK for this package is installed and sourced properly
if(NOT "$ENV{PUREGEV_ROOT}" STREQUAL "/opt/pleora/ebus_sdk")
  message(STATUS "+++++++++ Did not find PUREGEV_ROOT, not building seneka_sony_camera +++++++++")
  return()
else(NOT "$ENV{PUREGEV_ROOT}" STREQUAL "/opt/pleora/ebus_sdk")
  message(STATUS "+++++++++ Did find PUREGEV_ROOT, building seneka_sony_camera +++++++++")
endif(NOT "$ENV{PUREGEV_ROOT}" STREQUAL "/opt/pleora/ebus_sdk")

## Check which architecture we have
if(“${CMAKE_SYSTEM_PROCESSOR}” STREQUAL “x86_64”)
  set(MYARCH "x86_64")
else(“${CMAKE_SYSTEM_PROCESSOR}” STREQUAL “x86_64”)
  set(MYARCH "i386")
endif(“${CMAKE_SYSTEM_PROCESSOR}” STREQUAL “x86_64”)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  seneka_srv
)

#find_package(Boost REQUIRED thread)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES dummy_pkg
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_transport
    cv_bridge
    seneka_srv
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_definitions(-D_UNIX_)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /opt/pleora/ebus_sdk/include
  ${catkin_INCLUDE_DIRS}
)

# find Pleora Libraries
LINK_DIRECTORIES(/opt/pleora/ebus_sdk/lib)
if(“${MYARCH}” STREQUAL “x86_64”)
  LINK_DIRECTORIES(/opt/pleora/ebus_sdk/lib/genicam/bin/Linux64_x64)
else(“${MYARCH}” STREQUAL “x86_64”)
  LINK_DIRECTORIES(/opt/pleora/ebus_sdk/lib/genicam/bin/Linux32_i86)
endif(“${MYARCH}” STREQUAL “x86_64”)

# set library collections for easier linking
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

## Declare a cpp executable
add_executable(seneka_sony_camera src/sony_camera_node.cpp)

## make sure to have the correct order for building
add_dependencies(seneka_sony_camera seneka_srv_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(seneka_sony_camera 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EBUS_LIB}
  ${GENICAM_LIB}
)

