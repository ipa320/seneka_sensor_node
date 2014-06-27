cmake_minimum_required(VERSION 2.8.3)
project(seneka_socketcan)


set(CMAKE_BUILD_TYPE Release)


find_package(
  catkin REQUIRED COMPONENTS
    roscpp
)


catkin_package(
  INCLUDE_DIRS
  ros/src
  common/include
  CATKIN_DEPENDS
    roscpp
)


include_directories(
  ros/src
  common/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ros/src/socketCAN.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)