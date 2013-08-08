cmake_minimum_required(VERSION 2.8.3)
project(videoManager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  rospy
  cv_bridge
  roscpp
  image_transport
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS 
  thread
  serialization
  iostreams
  system
)

find_package(OpenCV REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    getLiveStream.srv
    getSnapShots.srv
    getVideo.srv
)

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
  INCLUDE_DIRS src
#  LIBRARIES dummy_pkg
 CATKIN_DEPENDS
  std_msgs
  sensor_msgs
  rospy
  cv_bridge
  roscpp
  image_transport
  message_runtime
 DEPENDS
  opencv
  Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(videoManager src/videoManager.cpp)
add_executable(vTester src/vTester.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(videoManager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_link_libraries(vTester
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
 install(TARGETS videoManager videoManager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

