cmake_minimum_required(VERSION 3.0.2)
project(rtsp_stream)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 编译地图RTSP推流节点
add_executable(map_rtsp_publisher src/map_rtsp_publisher.cpp)
target_link_libraries(map_rtsp_publisher 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# 编译推流监听程序（独立版本，不依赖ROS）
add_executable(stream_listener src/stream_listener.cpp)
# 独立版本不链接 ROS 库

# 安装可执行文件
install(TARGETS map_rtsp_publisher stream_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/tuiliu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# 安装web文件
install(DIRECTORY web/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web
)
