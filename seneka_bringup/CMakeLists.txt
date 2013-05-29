cmake_minimum_required(VERSION 2.8.3)
project(seneka_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  seneka_config
  seneka_environments
  sensor_placement
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES seneka
  CATKIN_DEPENDS
    map_server
    seneka_config
    seneka_environments
    sensor_placement
    rviz
  #DEPENDS system_lib
)
## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   launch/rviz.launch
   launch/sensor_placement.launch
   components/sensor_placement.xml
   # myfile2
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

