cmake_minimum_required(VERSION 2.8.3)
project(seneka_leg)


set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED
  roscpp
  seneka_socketcan
)


catkin_package(
  INCLUDE_DIRS
  common/include
  CATKIN_DEPENDS
    roscpp
    seneka_socketcan
)


include_directories(
  common/include
  ${catkin_INCLUDE_DIRS}
)