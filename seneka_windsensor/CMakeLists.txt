cmake_minimum_required(VERSION 2.8.3)
project(seneka_windsensor)

# set build type
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  std_msgs
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
    msg
  FILES
    WindData.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
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
    std_msgs
    message_runtime
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
add_executable(seneka_windsensor ros/src/seneka_windsensor.cpp common/src/windsensor.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(seneka_windsensor ${PROJECT_NAME}_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(seneka_windsensor
   ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS seneka_windsensor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
