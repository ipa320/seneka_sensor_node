cmake_minimum_required(VERSION 2.8.3)
project(sensor_placement)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rosconsole
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf
  tf_conversions
  std_srvs
  message_generation
)

add_service_files(
  FILES
  polygon_offset.srv
)

add_message_files(
  FILES
  PolygonStamped_array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
    common/include
    third_party/clipper_library/include
    CATKIN_DEPENDS message_runtime
#  LIBRARIES seneka
  CATKIN_DEPENDS
    roslib
    roscpp
    rosconsole
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf
    std_srvs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(include/sensor_placement
  common/include/sensor_placement
  third_party/clipper_library/include/clipper
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
# add_executable(seneka_node src/seneka_node.cpp)
add_executable(sensor_placement_node 	src/sensor_placement_node.cpp
					common/src/particle.cpp
					common/src/seneka_utilities.cpp
					common/src/sensor_model.cpp
					common/src/greedySearch.cpp
					third_party/clipper_library/src/clipper.cpp
					)
add_executable(sensor_placement_test	test/sensor_placement_test_publisher.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(sensor_placement_node
   ${catkin_LIBRARIES}
 )
 target_link_libraries(sensor_placement_test
   ${catkin_LIBRARIES}
 )

## Mark executables and/or libraries for installation
 install(TARGETS sensor_placement_node sensor_placement_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

