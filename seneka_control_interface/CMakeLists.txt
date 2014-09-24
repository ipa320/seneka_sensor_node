cmake_minimum_required(VERSION 2.8.3)
project(seneka_control_interface)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  roscpp
  seneka_socketcan
  seneka_laser_scan
#  seneka_turret
  seneka_leg
#  seneka_tilt
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    seneka_socketcan
    seneka_laser_scan
#    seneka_turret
    seneka_leg
#    seneka_tilt
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS seneka_control_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
