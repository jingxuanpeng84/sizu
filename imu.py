#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 纯推算 2D x, y, yaw + ZUPT + EMA 滤波 (归一化 IMU 数据)
"""

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf

IMU_TOPIC = "/livox/imu"
ODOM_TOPIC = "/imu_odom"
INIT_SAMPLE = 100
G = 9.80665
OMEGA_SCALE = math.radians(200.0)  # 归一化角速度对应 ±200°/s

class IMUOdometry:
    def __init__(self):
        self.odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=1)
        self.first = True
        self.prev_time = None

        # 位姿状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # 偏置
        self.gyro_bias_z = 0.0
        self.acc_bias_x = 0.0
        self.acc_bias_y = 0.0
        self.init_buffer = []

        # 静止判定阈值
        self.ZUPT_ACC_THRESHOLD = 0.08 * G   # m/s²
        self.ZUPT_GYRO_THRESHOLD = 0.01 * OMEGA_SCALE  # rad/s

        # EMA 滤波系数
        self.alpha = 0.2
        self.ax_filt = 0.0
        self.ay_filt = 0.0

    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.to_sec()
        if self.first:
            self.prev_time = t
            self.first = False
        dt = t - self.prev_time
        self.prev_time = t
        if dt <= 0.0:
            return

        # -------------------------------
        # 偏置初始化
        # -------------------------------
        if len(self.init_buffer) < INIT_SAMPLE:
            self.init_buffer.append(msg)
            if len(self.init_buffer) == INIT_SAMPLE:
                self.gyro_bias_z = np.mean([m.angular_velocity.z for m in self.init_buffer])
                self.acc_bias_x = np.mean([m.linear_acceleration.x for m in self.init_buffer])
                self.acc_bias_y = np.mean([m.linear_acceleration.y for m in self.init_buffer])
                rospy.loginfo(f"[IMU Odometry] 初始化完成: gyro_bias_z={self.gyro_bias_z:.5f}, "
                              f"acc_bias_x={self.acc_bias_x:.5f}, acc_bias_y={self.acc_bias_y:.5f}")
            return

        # -------------------------------
        # 去偏置并还原单位
        # -------------------------------
        wz = (msg.angular_velocity.z - self.gyro_bias_z) * OMEGA_SCALE
        ax = (msg.linear_acceleration.x - self.acc_bias_x) * G
        ay = (msg.linear_acceleration.y - self.acc_bias_y) * G

        # -------------------------------
        # EMA 滤波
        # -------------------------------
        self.ax_filt = self.alpha * ax + (1 - self.alpha) * self.ax_filt
        self.ay_filt = self.alpha * ay + (1 - self.alpha) * self.ay_filt

        # -------------------------------
        # 静止判定 (ZUPT)
        # -------------------------------
        is_stopped = (abs(self.ax_filt) < self.ZUPT_ACC_THRESHOLD and
                      abs(self.ay_filt) < self.ZUPT_ACC_THRESHOLD and
                      abs(wz) < self.ZUPT_GYRO_THRESHOLD)
        if is_stopped:
            self.vx = 0.0
            self.vy = 0.0
            wz = 0.0

        # -------------------------------
        # 运动积分
        # -------------------------------
        else:
            # yaw 积分
            self.yaw += wz * dt

            # body -> world
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            ax_world = cos_yaw * self.ax_filt - sin_yaw * self.ay_filt
            ay_world = sin_yaw * self.ax_filt + cos_yaw * self.ay_filt

            # 积分速度
            self.vx += ax_world * dt
            self.vy += ay_world * dt

            # 积分位置
            self.x += self.vx * dt
            self.y += self.vy * dt

        # -------------------------------
        # 发布 Odometry
        # -------------------------------
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # -------------------------------
        # 打印信息，每 10 帧打印一次
        # -------------------------------
        if int(msg.header.stamp.to_nsec() / 1e7) % 10 == 0:
            rospy.loginfo(f"[IMU Odometry] x={self.x:.4f} m, y={self.y:.4f} m, "
                          f"yaw={math.degrees(self.yaw):.2f} deg, "
                          f"vx={self.vx:.4f} m/s, vy={self.vy:.4f} m/s, "
                          f"wz={wz:.4f} rad/s, stopped={is_stopped}")


if __name__ == "__main__":
    rospy.init_node("imu_odom_node")
    imu_odom = IMUOdometry()
    rospy.Subscriber(IMU_TOPIC, Imu, imu_odom.imu_callback)
    rospy.loginfo("[IMU Odometry] 节点启动，等待 IMU 数据...")
    rospy.spin()
