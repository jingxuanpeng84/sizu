#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SLAM端地图保存监听服务
监听来自服务器端的保存地图请求，执行保存操作并退出建图程序
"""

import socket
import subprocess
import signal
import os
import sys
import time
import glob
import re
from datetime import datetime

# 配置
LISTEN_PORT = 9999  # 监听端口
MAP_SAVE_PATH = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD/scans"
MAP_DIR = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD"

def get_next_map_number():
    """
    扫描PCD目录，找到最大的地图序号，返回下一个序号
    例如：已有 scans1.pgm, scans2.pgm，返回 3
    """
    try:
        # 查找所有 scans*.pgm 文件
        pattern = os.path.join(MAP_DIR, "scans*.pgm")
        existing_maps = glob.glob(pattern)
        
        if not existing_maps:
            print("未找到已有地图文件，将从 scans1 开始")
            return 1
        
        # 提取所有序号
        numbers = []
        for map_file in existing_maps:
            basename = os.path.basename(map_file)  # 例如：scans1.pgm
            # 匹配 scans 后面的数字
            match = re.search(r'scans(\d+)\.pgm$', basename)
            if match:
                numbers.append(int(match.group(1)))
        
        if not numbers:
            print("未找到有效的地图序号，将从 scans1 开始")
            return 1
        
        max_num = max(numbers)
        next_num = max_num + 1
        print(f"找到最大序号: {max_num}，下一个序号: {next_num}")
        return next_num
        
    except Exception as e:
        print(f"获取地图序号时出错: {e}")
        return 1

def save_map_and_exit():
    """保存2D地图并退出建图程序"""
    print(f"[{datetime.now()}] 开始保存地图...")
    
    # 1. 获取下一个地图序号
    map_number = get_next_map_number()
    map_save_path = f"{MAP_SAVE_PATH}{map_number}"
    
    print(f"地图将保存为: {map_save_path}.pgm 和 {map_save_path}.yaml")
    
    # 2. 保存2D地图
    # 直接使用 subprocess.Popen 并等待完成
    save_cmd = [
        'bash', '-c',
        f'source /opt/ros/noetic/setup.bash && rosrun map_server map_saver map:=/projected_map -f {map_save_path}'
    ]
    
    print(f"执行命令: rosrun map_server map_saver map:=/projected_map -f {map_save_path}")
    print("等待地图保存...")
    
    try:
        # 使用 Popen 以便实时看到输出
        process = subprocess.Popen(
            save_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # 等待最多30秒
        try:
            stdout, stderr = process.communicate(timeout=30)
            
            if process.returncode == 0:
                # 检查文件是否真的生成了
                pgm_file = f"{map_save_path}.pgm"
                yaml_file = f"{map_save_path}.yaml"
                
                if os.path.exists(pgm_file) and os.path.exists(yaml_file):
                    pgm_size = os.path.getsize(pgm_file)
                    print(f"✓ 2D地图保存成功: {pgm_file} ({pgm_size} bytes)")
                    print(f"✓ YAML文件保存成功: {yaml_file}")
                    if stdout:
                        print(f"输出: {stdout}")
                    
                    # 保存最新地图序号到文件，供upload_map_ftp.py读取
                    latest_map_file = os.path.join(MAP_DIR, ".latest_map_number")
                    with open(latest_map_file, 'w') as f:
                        f.write(str(map_number))
                    print(f"✓ 最新地图序号已保存: {map_number}")
                    
                else:
                    print(f"⚠ 命令执行完成但文件未生成")
                    print(f"标准输出: {stdout}")
                    print(f"错误输出: {stderr}")
                    return False
            else:
                print(f"✗ 2D地图保存失败")
                print(f"返回码: {process.returncode}")
                print(f"标准输出: {stdout}")
                print(f"错误输出: {stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            print("✗ 保存2D地图超时（30秒）")
            process.kill()
            stdout, stderr = process.communicate()
            print(f"标准输出: {stdout}")
            print(f"错误输出: {stderr}")
            return False
            
    except Exception as e:
        print(f"✗ 保存2D地图出错: {e}")
        return False
    
    # 3. 等待文件写入完成
    time.sleep(0.5)
    
    # 4. 发送信号给建图程序（优雅退出，等同于 Ctrl+C）
    print("发送优雅退出信号给建图程序...")
    
    # 优雅退出策略：
    # 1. SIGINT (Ctrl+C) - 最优雅，程序可以清理资源并保存数据
    # 2. SIGTERM - 次优雅，程序可以捕获并清理
    # 3. rosnode kill - ROS 官方方式，发送 SIGINT
    
    kill_commands = [
        # 方式1: 通过 ROS 节点名称（最推荐，ROS 官方方式）
        # rosnode kill 内部发送 SIGINT 信号
        ("rosnode kill /laserMapping", "ROS节点终止（SIGINT）"),
        
        # 方式2: 直接发送 SIGINT 信号（等同于 Ctrl+C）
        ("pkill -SIGINT -f 'laserMapping'", "发送SIGINT信号（Ctrl+C）"),
        
        # 方式3: 通过完整进程名匹配
        ("pkill -SIGINT -f 'fastlio.*laserMapping'", "正则匹配进程名（SIGINT）"),
        
        # 方式4: 发送 SIGTERM 信号（备选方案）
        ("pkill -SIGTERM -f 'laserMapping'", "发送SIGTERM信号"),
        
        # 方式5: 如果配置了 sudo 无密码（最后尝试）
        ("sudo -n pkill -SIGINT -f 'laserMapping'", "sudo权限（SIGINT）"),
    ]
    
    killed = False
    for cmd, description in kill_commands:
        try:
            print(f"尝试: {description}")
            print(f"  命令: {cmd}")
            result = subprocess.run(
                ['bash', '-c', f'source /opt/ros/noetic/setup.bash && {cmd}'],
                shell=False,
                timeout=5,
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0 or 'killed' in result.stdout.lower():
                print(f"✓ 成功发送优雅退出信号")
                if result.stdout:
                    print(f"  输出: {result.stdout.strip()}")
                killed = True
                break
            else:
                print(f"  返回码: {result.returncode}")
                if result.stderr and 'sudo' not in result.stderr and 'not found' not in result.stderr.lower():
                    print(f"  错误: {result.stderr.strip()}")
                    
        except subprocess.TimeoutExpired:
            print(f"  超时（5秒）")
            continue
        except Exception as e:
            print(f"  执行失败: {e}")
            continue
    
    if not killed:
        print("⚠ 未能自动终止建图程序")
        print("提示: 建图程序可能以 root 用户运行，或进程名称不匹配")
        print("建议: 在建图终端按 Ctrl+C 手动退出以保存 PCD 点云")
        print("说明: 2D 地图（.pgm + .yaml）已成功保存")
        # 不返回 False，因为 2D 地图已经保存成功
    else:
        print("  建图程序将保存3D点云（PCD）并退出...")
        print("  等待 PCD 保存完成（约 5-10 秒）...")
        
        # 等待 PCD 文件生成或更新
        scans_pcd = os.path.join(MAP_DIR, "scans.pcd")
        pcd_saved = False
        
        # 等待最多 15 秒
        for i in range(15):
            time.sleep(1)
            
            # 检查 scans.pcd 是否存在且最近被修改
            if os.path.exists(scans_pcd):
                mtime = os.path.getmtime(scans_pcd)
                if time.time() - mtime < 10:  # 10秒内修改过
                    print(f"✓ 检测到 PCD 文件更新: scans.pcd")
                    pcd_saved = True
                    
                    # 重命名为带序号的文件名
                    new_pcd_name = f"scans{map_number}.pcd"
                    new_pcd_path = os.path.join(MAP_DIR, new_pcd_name)
                    
                    try:
                        # 如果目标文件已存在，先删除
                        if os.path.exists(new_pcd_path):
                            os.remove(new_pcd_path)
                            print(f"  已删除旧文件: {new_pcd_name}")
                        
                        # 重命名
                        os.rename(scans_pcd, new_pcd_path)
                        pcd_size = os.path.getsize(new_pcd_path)
                        print(f"✓ PCD 文件已重命名: {new_pcd_name} ({pcd_size} bytes)")
                    except Exception as e:
                        print(f"⚠ PCD 文件重命名失败: {e}")
                    
                    break
            
            if (i + 1) % 3 == 0:
                print(f"  等待中... ({i + 1}/15 秒)")
        
        if not pcd_saved:
            print("⚠ 未检测到 PCD 文件生成或更新")
            print("  可能原因:")
            print("  1. PCD 保存时间较长，请稍后手动检查")
            print("  2. pcd_save_en 参数未启用（需要在 launch 文件中设置为 true）")
            print("  3. 建图时间太短，点云数据不足")
            print("  4. 建图程序未正常退出")
    
    return True

def start_listener():
    """启动监听服务"""
    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(('0.0.0.0', LISTEN_PORT))
        print(f"=== SLAM端地图保存监听服务已启动 ===")
        print(f"监听端口: {LISTEN_PORT}")
        print(f"地图保存路径: {MAP_SAVE_PATH}")
        print(f"等待服务器端的保存请求...\n")
        
        while True:
            try:
                # 接收数据
                data, addr = sock.recvfrom(1024)
                message = data.decode('utf-8').strip()
                
                print(f"\n[{datetime.now()}] 收到来自 {addr[0]}:{addr[1]} 的请求")
                print(f"消息内容: {message}")
                
                # 处理保存地图请求
                if message == "SAVE_MAP":
                    success = save_map_and_exit()
                    
                    # 发送响应
                    if success:
                        response = "OK:Map saved successfully"
                        print(f"\n✓ 地图保存完成")
                    else:
                        response = "ERROR:Failed to save map"
                        print(f"\n✗ 地图保存失败")
                    
                    sock.sendto(response.encode('utf-8'), addr)
                    print(f"已发送响应到 {addr[0]}:{addr[1]}: {response}\n")
                    
                    # 保存完成后继续监听，不退出
                    # 这样可以多次保存地图而不需要重启服务
                    print("等待下一次保存请求...\n")
                    
                else:
                    print(f"⚠ 未知命令: {message}\n")
                    sock.sendto(b"ERROR:Unknown command", addr)
                    
            except KeyboardInterrupt:
                print("\n\n收到中断信号，正在关闭监听服务...")
                break
            except Exception as e:
                print(f"✗ 处理请求时出错: {e}\n")
                
    except Exception as e:
        print(f"✗ 启动监听服务失败: {e}")
        return 1
    finally:
        sock.close()
        print("监听服务已关闭")
    
    return 0

if __name__ == "__main__":
    sys.exit(start_listener())
