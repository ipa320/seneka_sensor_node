cmake_minimum_required(VERSION 2.8.3)
project(seneka_laser_scan)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  laser_assembler
)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp laser_assembler
# DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
