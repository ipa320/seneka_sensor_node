cmake_minimum_required(VERSION 2.8.3)
project(seneka_control_interface)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  roscpp
  actionlib
  genmsg
  actionlib_msgs
  seneka_socketcan
  seneka_laser_scan
  seneka_leg
  seneka_msg
  trajectory_msgs
)

add_action_files(DIRECTORY action FILES JointTrajectory.action)
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    seneka_socketcan
    seneka_laser_scan
    seneka_leg
    control_msgs
    seneka_msg
    trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS seneka_control_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
