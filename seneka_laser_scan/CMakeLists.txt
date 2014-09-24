cmake_minimum_required(VERSION 2.8.3)
project(seneka_laser_scan)

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

#add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} laser_assembler_gencpp)
