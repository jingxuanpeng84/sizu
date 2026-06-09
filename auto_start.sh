#!/bin/bash

# 等待系统启动
sleep 3

# 切换到工作空间
cd /home/orangepi/slam_ws

# 确保 .ros 日志目录存在
mkdir -p /home/orangepi/.ros/log

# 清理可能残留的 ROS 进程
killall -9 rosmaster roscore 2>/dev/null
sleep 1

# 先启动 ROS master（在后台独立运行）
source ./devel/setup.bash
roscore &

# 等待 ROS master 完全启动
echo "等待 ROS master 启动..."
sleep 8

# 验证 ROS master 是否就绪
until rostopic list &>/dev/null; do
    echo "等待 ROS master 就绪..."
    sleep 2
done
echo "ROS master 已就绪！"

# 启动雷达驱动（在新终端打开，并保持终端窗口）
xfce4-terminal --title="Livox Driver" --hold -e "bash -c 'source /home/orangepi/slam_ws/devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch'" &

# 等待雷达节点初始化
sleep 10

# 启动地图定位模块（在新终端打开，并保持终端窗口）
xfce4-terminal --title="Map Switch" --hold -e "bash -c 'source /home/orangepi/slam_ws/devel/setup.bash; roslaunch map_switch project.launch'" &

# 等待定位模块初始化并发布/Odometry话题
echo "等待定位模块初始化..."
sleep 5

# 验证/Odometry话题是否就绪
until rostopic list | grep -q "/Odometry"; do
    echo "等待 /Odometry 话题就绪..."
    sleep 2
done
echo "/Odometry 话题已就绪！"

# 启动位置服务器（在新终端打开，并保持终端窗口）
xfce4-terminal --title="Position Server" --hold -e "bash -c 'source /home/orangepi/slam_ws/devel/setup.bash; roslaunch goal_sender position_server_newdevice.launch'" &

echo "所有节点启动完成！"

