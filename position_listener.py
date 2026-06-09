#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位置监听服务
监听端口: 7070 (UDP)
功能: 接收HTTP服务器的位置查询请求，返回当前SLAM位置数据
"""

import socket
import struct
import threading
import time
import yaml
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# ==================== 配置 ====================
LISTEN_PORT = 7070
LISTEN_IP = "0.0.0.0"
ODOM_TOPIC = "/Odometry"
YAML_FILE = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD/scans_507_in.yaml"

# ==================== 全局变量 ====================
current_position = {
    'x': 0.0,
    'y': 0.0,
    'yaw': 0.0,
    'px': 0,
    'py': 0,
    'valid': False
}

map_params = {
    'resolution': 0.02,
    'origin_x': -5.96,
    'origin_y': -6.02,
    'width': 835,
    'height': 464
}

position_lock = threading.Lock()

# ==================== 地图参数加载 ====================
def load_map_params():
    """从YAML文件加载地图参数"""
    global map_params
    try:
        with open(YAML_FILE, 'r') as f:
            data = yaml.safe_load(f)
            if data:
                map_params['resolution'] = data.get('resolution', 0.02)
                map_params['origin_x'] = data['origin'][0]
                map_params['origin_y'] = data['origin'][1]
                # 注意：width和height需要从image字段获取，如果没有则使用默认值
                print(f"[PositionListener] 地图参数已加载:")
                print(f"  - 分辨率: {map_params['resolution']} m/pixel")
                print(f"  - 原点: ({map_params['origin_x']}, {map_params['origin_y']})")
                print(f"  - 尺寸: {map_params['width']} x {map_params['height']} pixels")
                return True
    except Exception as e:
        print(f"[PositionListener] 加载地图参数失败: {e}")
        print(f"[PositionListener] 使用默认地图参数")
    return False

# ==================== 坐标转换 ====================
def world_to_pixel(world_x, world_y):
    """将世界坐标转换为像素坐标"""
    offset_x = world_x - map_params['origin_x']
    offset_y = world_y - map_params['origin_y']
    
    pixel_x = int(offset_x / map_params['resolution'])
    pixel_y = map_params['height'] - int(offset_y / map_params['resolution']) - 1
    
    # 边界检查
    pixel_x = max(0, min(pixel_x, map_params['width'] - 1))
    pixel_y = max(0, min(pixel_y, map_params['height'] - 1))
    
    return pixel_x, pixel_y

# ==================== ROS回调 ====================
def odom_callback(msg):
    """里程计数据回调"""
    global current_position
    
    with position_lock:
        # 更新世界坐标
        current_position['x'] = msg.pose.pose.position.x
        current_position['y'] = msg.pose.pose.position.y
        
        # 计算yaw角
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        current_position['yaw'] = yaw
        
        # 计算像素坐标
        px, py = world_to_pixel(current_position['x'], current_position['y'])
        current_position['px'] = px
        current_position['py'] = py
        
        current_position['valid'] = True

# ==================== UDP服务器 ====================
def handle_udp_request(sock):
    """处理UDP请求"""
    print(f"[PositionListener] UDP服务器启动在 {LISTEN_IP}:{LISTEN_PORT}")
    
    while True:
        try:
            # 接收数据
            data, addr = sock.recvfrom(1024)
            
            if len(data) < 24:  # frame头部至少24字节
                continue
            
            # 解析frame头部（简化版本，只检查type）
            # struct frame { int source; int dest; int type; int len_data; char data[]; }
            source, dest, frame_type, len_data = struct.unpack('iiii', data[:16])
            
            print(f"[PositionListener] 收到请求 from {addr[0]}:{addr[1]}, type={frame_type}")
            
            # 处理INQUIRY请求 (type=3)
            if frame_type == 3:
                with position_lock:
                    pos = current_position.copy()
                
                # 构造响应数据
                # POSITION::receiveFrameStruct { int frame_type; unsigned long seq; position_data data; }
                # position_data { double x, y, yaw; int px, py; }
                response_data = struct.pack(
                    'iQ ddd ii',  # frame_type(int), seq(ulong), x(double), y(double), yaw(double), px(int), py(int)
                    1,  # frame_type = 1 (RESPONSE)
                    0,  # seq = 0
                    pos['x'],
                    pos['y'],
                    pos['yaw'],
                    pos['px'],
                    pos['py']
                )
                
                # 构造完整的frame包
                # struct frame { int source; int dest; int type; int len_data; char data[]; }
                frame_header = struct.pack(
                    'iiii',
                    2,  # source = LADDAR_OBJ (假设为2)
                    1,  # dest = HTTPSERVER_OBJ (假设为1)
                    1,  # type = RESPONSE
                    len(response_data)
                )
                
                response = frame_header + response_data
                
                # 发送响应
                sock.sendto(response, addr)
                
                print(f"[PositionListener] 已响应位置数据到 {addr[0]}:{addr[1]}")
                print(f"  - 世界坐标: ({pos['x']:.4f}, {pos['y']:.4f})")
                print(f"  - 像素坐标: ({pos['px']}, {pos['py']})")
                
        except Exception as e:
            print(f"[PositionListener] 处理请求时出错: {e}")
            import traceback
            traceback.print_exc()

# ==================== 主函数 ====================
def main():
    print("=== 位置监听服务启动 ===")
    
    # 加载地图参数
    load_map_params()
    
    # 初始化ROS节点
    try:
        rospy.init_node('position_listener', anonymous=True)
        print("[PositionListener] ROS节点已初始化")
    except Exception as e:
        print(f"[PositionListener] ROS初始化失败: {e}")
        print("[PositionListener] 将以非ROS模式运行（位置数据将为0）")
    
    # 订阅里程计话题
    try:
        rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback)
        print(f"[PositionListener] 已订阅话题: {ODOM_TOPIC}")
    except Exception as e:
        print(f"[PositionListener] 订阅话题失败: {e}")
    
    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    
    # 启动UDP服务器线程
    udp_thread = threading.Thread(target=handle_udp_request, args=(sock,), daemon=True)
    udp_thread.start()
    
    print("[PositionListener] 位置监听服务已启动，等待请求...")
    print(f"[PositionListener] 监听地址: {LISTEN_IP}:{LISTEN_PORT}")
    
    # 定期打印位置信息
    count = 0
    try:
        while True:
            time.sleep(1)
            count += 1
            if count % 10 == 0:
                with position_lock:
                    if current_position['valid']:
                        print(f"[PositionListener] 当前位置: "
                              f"世界坐标=({current_position['x']:.4f}, {current_position['y']:.4f}), "
                              f"像素坐标=({current_position['px']}, {current_position['py']})")
                    else:
                        print("[PositionListener] 等待位置数据...")
    except KeyboardInterrupt:
        print("\n[PositionListener] 收到中断信号，正在关闭...")
    finally:
        sock.close()
        print("[PositionListener] 位置监听服务已关闭")

if __name__ == "__main__":
    main()
