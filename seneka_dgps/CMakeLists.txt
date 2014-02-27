cmake_minimum_required(VERSION 2.8.3)
project(seneka_dgps)

# set build type
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  sensor_msgs
  diagnostic_msgs
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
  INCLUDE_DIRS 
    common/include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    diagnostic_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(seneka_dgps ros/src/seneka_dgps.cpp common/src/Dgps.cpp common/src/SerialIO.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(seneka_dgps
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html
## Mark executables and/or libraries for installation
install(TARGETS seneka_dgps
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

