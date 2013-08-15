cmake_minimum_required(VERSION 2.8.3)
project(seneka_termo_video_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  std_msgs
  sensor_msgs
  cv_bridge
  roscpp
  image_transport
  optris_drivers
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

#######################################
## Declare ROS messages and services ##
#######################################


## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
    srv
  FILES
    getLiveStream.srv
    getSnapShots.srv
    getVideo.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    std_srvs
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
# INCLUDE_DIRS 
# LIBRARIES
 CATKIN_DEPENDS
   std_msgs
   sensor_msgs
   cv_bridge
   roscpp
   image_transport
   optris_drivers
   message_runtime
 DEPENDS 
   OpenCV
   Boost
   libudev-dev
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
add_executable(termo_video_manager src/termoVideoManager.cpp)
add_executable(termo_video_tester src/vTester.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(termo_video_manager ${PROJECT_NAME}_gencpp optris_drivers)
add_dependencies(termo_video_tester ${PROJECT_NAME}_gencpp optris_drivers)

## Specify libraries to link a library or executable target against
target_link_libraries(termo_video_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libPIImager.a
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libImageProcessing.a
  udev
)
target_link_libraries(termo_video_tester
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libPIImager.a
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libImageProcessing.a
  udev
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS termo_video_manager termo_video_tester
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
