cmake_minimum_required(VERSION 2.8.3)
project(seneka_socketcan)


set(CMAKE_BUILD_TYPE Release)


find_package(
  catkin REQUIRED COMPONENTS
    roscpp
)


catkin_package(
  INCLUDE_DIRS
  common/include
  CATKIN_DEPENDS
    roscpp
)


include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
)