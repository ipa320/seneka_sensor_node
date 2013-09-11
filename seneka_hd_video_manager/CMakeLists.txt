cmake_minimum_required(VERSION 2.8.3)
project(seneka_hd_video_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  std_srvs
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
#add_service_files(
#  DIRECTORY
#    srv
#  FILES
#    getLiveStream.srv
#    getSnapShots.srv
#    getVideo.srv
#)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#    std_msgs
#    std_srvs
#)

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
# INCLUDE_DIRS
# LIBRARIES
 CATKIN_DEPENDS
  std_msgs
  std_srvs
  sensor_msgs
  rospy
  cv_bridge
  roscpp
  image_transport
  message_runtime
 DEPENDS
  OpenCV
  Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
)

## Declare a cpp executable
add_executable(hd_video_manager_node src/hdVideoManager.cpp)
#add_executable(video_tester src/vTester.cpp)

add_dependencies(hd_video_manager_node ${PROJECT_NAME}_gencpp)
#add_dependencies(video_tester ${PROJECT_NAME}_gencpp)


## Specify libraries to link a library or executable target against
target_link_libraries(hd_video_manager_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  avformat
  avcodec
  swscale 
  avutil
  SDL
)
#target_link_libraries(video_tester
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBRARIES}
#  ${Boost_LIBRARIES}
#)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS video_manager_node video_tester
 install(TARGETS hd_video_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

