cmake_minimum_required(VERSION 2.8.3)
project(seneka_remote_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/Tower.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
