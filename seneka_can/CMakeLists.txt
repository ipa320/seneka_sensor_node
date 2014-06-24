cmake_minimum_required(VERSION 2.8.3)
project(seneka_can)


set(CMAKE_BUILD_TYPE Release)


find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    seneka_srv
)


catkin_package(
  INCLUDE_DIRS
    ros/include
  CATKIN_DEPENDS
    roscpp
    seneka_srv
)


include_directories(
  ros/include
  common/include
  ${catkin_INCLUDE_DIRS}
)


add_executable(seneka_can ros/src/seneka_can.cpp)

target_link_libraries(seneka_can
  ${catkin_LIBRARIES}
)

add_dependencies(seneka_can seneka_srv_gencpp)


add_executable(seneka_can_tester ros/src/seneka_can_tester.cpp)

target_link_libraries(seneka_can_tester
  ${catkin_LIBRARIES}
)

add_dependencies(seneka_can_tester seneka_srv_gencpp)


install(TARGETS seneka_can
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
