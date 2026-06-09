#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四元数转欧拉角计算器
"""

import math

def quaternion_to_euler(x, y, z, w):
    """
    将四元数转换为欧拉角 (Roll, Pitch, Yaw)
    
    参数:
        x, y, z, w: 四元数分量
    
    返回:
        (roll, pitch, yaw) 单位：弧度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def rad_to_deg(rad):
    """弧度转角度"""
    return rad * 180.0 / math.pi

# 给定的四元数
x = -0.001597330093864551
y = -0.006738392337992055
z = -0.637341659493524
w = 0.7705502265779378

print("=" * 60)
print("四元数转欧拉角计算")
print("=" * 60)
print(f"\n输入四元数:")
print(f"  x = {x}")
print(f"  y = {y}")
print(f"  z = {z}")
print(f"  w = {w}")

# 验证四元数归一化
magnitude = math.sqrt(x*x + y*y + z*z + w*w)
print(f"\n四元数模长: {magnitude:.6f}")
if abs(magnitude - 1.0) > 0.01:
    print("  ⚠️  警告: 四元数未归一化!")
else:
    print("  ✓ 四元数已归一化")

# 转换为欧拉角
roll, pitch, yaw = quaternion_to_euler(x, y, z, w)

print("\n" + "=" * 60)
print("欧拉角结果 (弧度)")
print("=" * 60)
print(f"  Roll  (横滚角): {roll:10.6f} rad")
print(f"  Pitch (俯仰角): {pitch:10.6f} rad")
print(f"  Yaw   (偏航角): {yaw:10.6f} rad")

print("\n" + "=" * 60)
print("欧拉角结果 (角度)")
print("=" * 60)
print(f"  Roll  (横滚角): {rad_to_deg(roll):10.4f}°")
print(f"  Pitch (俯仰角): {rad_to_deg(pitch):10.4f}°")
print(f"  Yaw   (偏航角): {rad_to_deg(yaw):10.4f}°")

print("\n" + "=" * 60)
print("机器人朝向解释")
print("=" * 60)
yaw_deg = rad_to_deg(yaw)
print(f"  偏航角 (Yaw): {yaw_deg:.2f}°")
print(f"  相对于X轴正方向: {yaw_deg:.2f}°")

# 判断朝向
if -22.5 <= yaw_deg < 22.5:
    direction = "东 (East, +X方向)"
elif 22.5 <= yaw_deg < 67.5:
    direction = "东北 (Northeast)"
elif 67.5 <= yaw_deg < 112.5:
    direction = "北 (North, +Y方向)"
elif 112.5 <= yaw_deg < 157.5:
    direction = "西北 (Northwest)"
elif 157.5 <= yaw_deg <= 180 or -180 <= yaw_deg < -157.5:
    direction = "西 (West, -X方向)"
elif -157.5 <= yaw_deg < -112.5:
    direction = "西南 (Southwest)"
elif -112.5 <= yaw_deg < -67.5:
    direction = "南 (South, -Y方向)"
elif -67.5 <= yaw_deg < -22.5:
    direction = "东南 (Southeast)"
else:
    direction = "未知"

print(f"  机器人朝向: {direction}")

print("\n" + "=" * 60)
print("坐标系说明")
print("=" * 60)
print("  ROS标准坐标系 (REP 103):")
print("    X轴: 前方 (Forward)")
print("    Y轴: 左方 (Left)")
print("    Z轴: 上方 (Up)")
print("    Yaw = 0°   表示朝向 +X 方向 (正前方)")
print("    Yaw = 90°  表示朝向 +Y 方向 (正左方)")
print("    Yaw = -90° 表示朝向 -Y 方向 (正右方)")
print("    Yaw = 180° 表示朝向 -X 方向 (正后方)")
print("=" * 60)
