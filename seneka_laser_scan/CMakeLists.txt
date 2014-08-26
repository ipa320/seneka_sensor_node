cmake_minimum_required(VERSION 2.8.3)
project(seneka_laser_scan)


set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  laser_assembler
)

catkin_package(
  INCLUDE_DIRS
  ros/include
  CATKIN_DEPENDS
    roscpp
    laser_assembler
)

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)
