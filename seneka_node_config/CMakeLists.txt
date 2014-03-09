cmake_minimum_required(VERSION 2.8.3)
project(seneka_node_config)

# set build type
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES seneka
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

