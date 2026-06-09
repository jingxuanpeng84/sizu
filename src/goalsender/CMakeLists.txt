cmake_minimum_required(VERSION 3.0.2)
project(goal_sender)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

## Catkin package configuration
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs tf
)

## Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../newdevice/include
  ${catkin_INCLUDE_DIRS}
)

## Build newdevice library (using local copies in src/network)
add_library(newdevice
  src/network/deviceBase.cpp
  src/network/networkBase.cpp
  src/network/tcpUdp.cpp
)

## Build laddar_server_node executable (integrated with laddar_server_newdevice)
add_executable(laddar_server_node
  src/laddar_server_newdevice.cpp
)
target_link_libraries(laddar_server_node
  newdevice
  ${catkin_LIBRARIES}
)

## Build laddar_server_mock executable (mock data for testing)
add_executable(laddar_server_mock
  laddar_server_mock.cpp
)
target_link_libraries(laddar_server_mock
  newdevice
  ${catkin_LIBRARIES}
)

## Build position_server_mock executable (mock position data for testing)
add_executable(position_server_mock
  position_server_mock.cpp
)
target_link_libraries(position_server_mock
  newdevice
  ${catkin_LIBRARIES}
)

## Build position_server_node executable (ROS data version)
add_executable(position_server_node
  position_server_newdevice.cpp
)
target_link_libraries(position_server_node
  newdevice
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS 
  laddar_server_node
  laddar_server_mock
  position_server_node
  position_server_mock
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
