#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from livox_ros_driver2.msg import CustomMsg  # Livox ROS2 driver 自定义消息

class Livox2Laser360Filtered:
    def __init__(self):
        rospy.init_node("livox_custom_to_2dscan_loop")

        # ROS 参数
        self.frame_id = rospy.get_param("~frame_id", "robot_body")
        self.range_min = rospy.get_param("~range_min", 0.1)
        self.range_max = rospy.get_param("~range_max", 10.0)
        self.num_points = rospy.get_param("~num_points", 720)
        self.z_min = rospy.get_param("~z_min", -0.5)
        self.z_max = rospy.get_param("~z_max", 2.0)
        self.reflectivity_min = rospy.get_param("~reflectivity_min", 0)
        self.lid_topic = rospy.get_param("~lid_topic", "/livox/lidar")
        self.max_buffer_size = rospy.get_param("~max_buffer_size", 10)  # 缓冲区最大帧数

        # 自动计算角度
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (self.angle_max - self.angle_min) / self.num_points

        # 缓冲区
        self.lidar_buffer = []

        # 发布 LaserScan
        self.pub = rospy.Publisher("/mid360/scan11", LaserScan, queue_size=1)

        # 订阅 Livox CustomMsg
        rospy.Subscriber(self.lid_topic, CustomMsg, self.livox_pcl_cbk, queue_size=1)

        rospy.loginfo("Livox CustomMsg to 2D LaserScan node started.")

    def livox_pcl_cbk(self, msg):
        timestamp_lidar = msg.header.stamp.to_sec()

        # 激光点投影到2D LaserScan
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float('inf')] * self.num_points

        for p in msg.points:
            x, y, z = p.x, p.y, p.z
            reflectivity = getattr(p, 'reflectivity', 0)

            if x == 0.0 and y == 0.0:
                continue
            if not (self.z_min <= z <= self.z_max):
                continue
            if reflectivity < self.reflectivity_min:
                continue

            r = math.hypot(x, y)
            if r < self.range_min or r > self.range_max:
                continue

            angle = math.atan2(y, x)
            index = int((angle - self.angle_min) / self.angle_increment)
            if 0 <= index < self.num_points:
                if r < scan.ranges[index]:
                    scan.ranges[index] = r

        # 限制缓冲区大小
        self.lidar_buffer.append(msg)
        if len(self.lidar_buffer) > self.max_buffer_size:
            self.lidar_buffer.pop(0)

        # 发布 LaserScan
        self.pub.publish(scan)

    def run_loop(self):
        rate_hz = rospy.get_param("~rate", 20)  # 循环频率
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            # 可以在这里处理缓冲区或做统计
            rate.sleep()


if __name__ == "__main__":
    node = Livox2Laser360Filtered()
    node.run_loop()
