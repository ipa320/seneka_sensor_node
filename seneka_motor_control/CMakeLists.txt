cmake_minimum_required(VERSION 2.8.3)
project(seneka_motor_control)


set(CMAKE_BUILD_TYPE Release)


find_package(
  catkin REQUIRED COMPONENTS
    roscpp
)


catkin_package(
  INCLUDE_DIRS
    ros/include
    common/include
  CATKIN_DEPENDS
    roscpp
)


include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)


add_executable(seneka_motor_control ros/src/seneka_motor_control.cpp)

target_link_libraries(seneka_motor_control
  ${catkin_LIBRARIES}
)


install(TARGETS seneka_motor_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
