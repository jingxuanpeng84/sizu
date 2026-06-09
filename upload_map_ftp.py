#!/usr/bin/env python3
"""
FTP 地图文件上传监听服务
监听来自服务器端的上传地图请求，将保存的地图文件上传到 FTP 服务器
支持自动上传最新保存的带序号地图（.pgm + .yaml）
支持列出所有地图和选择上传指定地图
"""

from ftplib import FTP
import sys
import os
import glob
import re
import socket
import json
from datetime import datetime

# 配置
LISTEN_PORT = 9998  # 监听端口（与 save_map_listener.py 不同）
MAP_SAVE_PATH = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD/scans"
MAP_DIR = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD"

# FTP 配置
FTP_SERVER_IP = "192.168.110.151"
FTP_PORT = 2121
FTP_USERNAME = "anonymous"
FTP_PASSWORD = ""

def get_all_maps():
    """
    扫描PCD目录，返回所有地图文件信息（包括带序号和不带序号的）
    
    返回:
        [
            {
                "number": 1,  # 如果有序号
                "name": "scans1",  # 地图名称（不含扩展名）
                "pgm_file": "scans1.pgm",
                "yaml_file": "scans1.yaml",
                "pgm_size": 2345678,
                "yaml_size": 123,
                "pgm_exists": True,
                "yaml_exists": True,
                "modified_time": "2025-05-07 14:30:22"
            },
            {
                "number": None,  # 无序号的地图
                "name": "scans",
                "pgm_file": "scans.pgm",
                "yaml_file": "scans.yaml",
                ...
            },
            ...
        ]
    """
    try:
        # 查找所有 .pgm 文件（不限制命名格式）
        pattern = os.path.join(MAP_DIR, "*.pgm")
        pgm_files = glob.glob(pattern)
        
        maps = []
        for pgm_file in pgm_files:
            basename = os.path.basename(pgm_file)
            
            # 提取文件名（不含扩展名）
            name_without_ext = os.path.splitext(basename)[0]
            
            # 尝试提取序号（如果有的话）
            match = re.search(r'(\w+?)(\d+)$', name_without_ext)
            if match:
                # 有序号：例如 scans1, map2, test123
                base_name = match.group(1)
                number = int(match.group(2))
            else:
                # 无序号：例如 scans, map, test
                base_name = name_without_ext
                number = None
            
            yaml_file = pgm_file.replace('.pgm', '.yaml')
            
            map_info = {
                "number": number,
                "name": name_without_ext,
                "pgm_file": basename,
                "yaml_file": os.path.basename(yaml_file),
                "pgm_size": os.path.getsize(pgm_file),
                "yaml_size": os.path.getsize(yaml_file) if os.path.exists(yaml_file) else 0,
                "pgm_exists": True,
                "yaml_exists": os.path.exists(yaml_file),
                "modified_time": datetime.fromtimestamp(os.path.getmtime(pgm_file)).strftime("%Y-%m-%d %H:%M:%S")
            }
            maps.append(map_info)
        
        # 排序：有序号的按序号排序，无序号的按名称排序，有序号的排在前面
        maps.sort(key=lambda x: (x["number"] is None, x["number"] if x["number"] is not None else 0, x["name"]))
        
        return maps
        
    except Exception as e:
        print(f"扫描地图文件时出错: {e}")
        return []

def get_latest_map_number():
    """
    获取最新保存的地图序号
    优先从 .latest_map_number 文件读取，如果不存在则扫描目录
    """
    latest_map_file = os.path.join(MAP_DIR, ".latest_map_number")
    
    # 方式1：从文件读取（save_map_listener.py 保存的）
    if os.path.exists(latest_map_file):
        try:
            with open(latest_map_file, 'r') as f:
                map_number = int(f.read().strip())
                print(f"从文件读取最新地图序号: {map_number}")
                return map_number
        except Exception as e:
            print(f"读取最新地图序号文件失败: {e}")
    
    # 方式2：扫描目录找最大序号
    maps = get_all_maps()
    if maps:
        max_num = max(map["number"] for map in maps)
        print(f"扫描目录找到最大序号: {max_num}")
        return max_num
    
    print("未找到任何地图文件")
    return None

def upload_map_ftp(filename):
    """
    上传地图文件到 FTP 服务器
    
    参数:
        filename: 要上传的文件路径
    
    返回:
        True: 上传成功
        False: 上传失败
    """
    if not os.path.exists(filename):
        print(f"错误：文件不存在 - {filename}")
        return False
    
    try:
        print(f"连接到 FTP 服务器 {FTP_SERVER_IP}:{FTP_PORT}...")
        ftp = FTP()
        ftp.connect(FTP_SERVER_IP, FTP_PORT, timeout=10)
        
        print(f"登录用户: {FTP_USERNAME}")
        ftp.login(FTP_USERNAME, FTP_PASSWORD)
        
        print(f"切换到二进制模式...")
        ftp.voidcmd('TYPE I')
        
        # 获取文件名
        basename = os.path.basename(filename)
        
        print(f"上传文件: {basename} ({os.path.getsize(filename)} bytes)")
        with open(filename, 'rb') as f:
            ftp.storbinary(f'STOR {basename}', f)
        
        print("✓ 上传成功！")
        
        ftp.quit()
        return True
        
    except Exception as e:
        print(f"✗ 上传失败: {e}")
        return False

def extract_map_id_from_filename(filename):
    """
    从文件名中提取地图编号
    
    规则:
    - scans1.yaml → 1
    - scans2.yaml → 2
    - scans_507_in.yaml → 507 (取中间的数字)
    - scans_507_out.yaml → 507
    - scans.yaml → 1 (默认)
    
    参数:
        filename: YAML文件名
    
    返回:
        地图编号 (int)
    """
    import re
    
    # 去除扩展名
    name_without_ext = filename.replace('.yaml', '').replace('.yml', '')
    
    # 尝试匹配各种模式
    patterns = [
        r'scans(\d+)$',           # scans1, scans2, scans123
        r'scans_(\d+)_',          # scans_507_in, scans_507_out
        r'scans_(\d+)$',          # scans_507
        r'map(\d+)$',             # map1, map2
        r'map_(\d+)',             # map_1, map_2
        r'(\d+)',                 # 任何数字
    ]
    
    for pattern in patterns:
        match = re.search(pattern, name_without_ext)
        if match:
            map_id = int(match.group(1))
            print(f"从文件名 '{filename}' 提取地图编号: {map_id}")
            return map_id
    
    # 如果没有匹配到，返回默认值1
    print(f"无法从文件名 '{filename}' 提取地图编号，使用默认值: 1")
    return 1


def upload_specific_map(map_identifier):
    """
    上传指定的地图（.pgm + .yaml）
    
    参数:
        map_identifier: 地图标识符，可以是：
            - 数字序号（如 1, 2, 3）
            - 地图名称（如 "scans", "scans1", "map"）
    
    返回:
        (success, uploaded_files): 
            success - 是否成功
            uploaded_files - 已上传的文件列表
    """
    # 尝试解析为数字序号
    try:
        map_number = int(map_identifier)
        # 如果是数字，尝试查找 scans{number}.pgm
        pgm_file = f"{MAP_SAVE_PATH}{map_number}.pgm"
        yaml_file = f"{MAP_SAVE_PATH}{map_number}.yaml"
        map_name = f"scans{map_number}"
    except ValueError:
        # 如果不是数字，作为地图名称处理
        map_name = str(map_identifier)
        pgm_file = os.path.join(MAP_DIR, f"{map_name}.pgm")
        yaml_file = os.path.join(MAP_DIR, f"{map_name}.yaml")
    
    if not os.path.exists(pgm_file):
        print(f"✗ 地图文件不存在: {pgm_file}")
        return False, []
    
    uploaded_files = []
    yaml_filename = os.path.basename(yaml_file)
    
    # 上传 .pgm 文件
    print(f"\n上传地图 {map_name}: {pgm_file}")
    if upload_map_ftp(pgm_file):
        uploaded_files.append(os.path.basename(pgm_file))
    else:
        print(f"✗ PGM文件上传失败")
        return False, uploaded_files
    
    # 上传 .yaml 文件
    if os.path.exists(yaml_file):
        print(f"上传配置文件: {yaml_file}")
        if upload_map_ftp(yaml_file):
            uploaded_files.append(yaml_filename)
        else:
            print(f"⚠ YAML文件上传失败，但PGM已上传")
            return False, uploaded_files
    else:
        print(f"⚠ YAML文件不存在: {yaml_file}")
        return False, uploaded_files
    
    # 写入标记文件，告诉 position_server 当前使用的地图
    marker_file = os.path.join(MAP_DIR, ".current_map_file")
    try:
        with open(marker_file, 'w') as f:
            f.write(yaml_filename)
        print(f"✓ 已写入标记文件: {marker_file} -> {yaml_filename}")
    except Exception as e:
        print(f"⚠ 写入标记文件失败: {e}")
    
    # 写入地图编号文件
    map_id = extract_map_id_from_filename(yaml_filename)
    map_id_file = os.path.join(MAP_DIR, ".current_map_id")
    try:
        with open(map_id_file, 'w') as f:
            f.write(str(map_id))
        print(f"✓ 已写入地图编号文件: {map_id_file} -> {map_id}")
    except Exception as e:
        print(f"⚠ 写入地图编号文件失败: {e}")
    
    print(f"✓ 地图 {map_name} 上传完成！")
    print(f"  已上传文件: {', '.join(uploaded_files)}")
    print(f"  地图编号: {map_id}")
    
    return True, uploaded_files

def upload_latest_map():
    """
    上传最新保存的地图文件（.pgm + .yaml）
    
    返回:
        (success, map_number, uploaded_files): 
            success - 是否成功
            map_number - 地图序号
            uploaded_files - 已上传的文件列表
    """
    # 获取最新地图序号
    map_number = get_latest_map_number()
    
    if map_number is None:
        print("✗ 无法确定最新地图序号")
        return False, None, []
    
    # 构建文件路径
    pgm_file = f"{MAP_SAVE_PATH}{map_number}.pgm"
    yaml_file = f"{MAP_SAVE_PATH}{map_number}.yaml"
    
    # 检查文件是否存在
    if not os.path.exists(pgm_file):
        print(f"✗ 地图文件不存在: {pgm_file}")
        return False, map_number, []
    
    if not os.path.exists(yaml_file):
        print(f"⚠ YAML文件不存在: {yaml_file}，将只上传PGM文件")
    
    # 上传文件
    uploaded_files = []
    yaml_filename = os.path.basename(yaml_file)
    
    # 上传 .pgm 文件
    print(f"\n开始上传地图文件: {pgm_file}")
    if upload_map_ftp(pgm_file):
        uploaded_files.append(os.path.basename(pgm_file))
    else:
        print(f"✗ PGM文件上传失败")
        return False, map_number, uploaded_files
    
    # 上传 .yaml 文件
    if os.path.exists(yaml_file):
        print(f"\n开始上传YAML文件: {yaml_file}")
        if upload_map_ftp(yaml_file):
            uploaded_files.append(yaml_filename)
        else:
            print(f"⚠ YAML文件上传失败，但PGM已上传")
            return False, map_number, uploaded_files
    else:
        print(f"⚠ YAML文件不存在，跳过上传")
        return False, map_number, uploaded_files
    
    # 写入标记文件，告诉 position_server 当前使用的地图
    marker_file = os.path.join(MAP_DIR, ".current_map_file")
    try:
        with open(marker_file, 'w') as f:
            f.write(yaml_filename)
        print(f"✓ 已写入标记文件: {marker_file} -> {yaml_filename}")
    except Exception as e:
        print(f"⚠ 写入标记文件失败: {e}")
    
    # 写入地图编号文件
    map_id = extract_map_id_from_filename(yaml_filename)
    map_id_file = os.path.join(MAP_DIR, ".current_map_id")
    try:
        with open(map_id_file, 'w') as f:
            f.write(str(map_id))
        print(f"✓ 已写入地图编号文件: {map_id_file} -> {map_id}")
    except Exception as e:
        print(f"⚠ 写入地图编号文件失败: {e}")
    
    print(f"\n✓ 地图上传完成！")
    print(f"  地图序号: {map_number}")
    print(f"  地图编号: {map_id}")
    print(f"  已上传文件: {', '.join(uploaded_files)}")
    
    return True, map_number, uploaded_files

def start_listener():
    """启动监听服务"""
    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(('0.0.0.0', LISTEN_PORT))
        print(f"=== FTP 地图上传监听服务已启动 ===")
        print(f"监听端口: {LISTEN_PORT}")
        print(f"地图目录: {MAP_DIR}")
        print(f"FTP 服务器: {FTP_SERVER_IP}:{FTP_PORT}")
        print(f"等待服务器端的上传请求...\n")
        
        while True:
            try:
                # 接收数据
                data, addr = sock.recvfrom(1024)
                message = data.decode('utf-8').strip()
                
                print(f"\n[{datetime.now()}] 收到来自 {addr[0]}:{addr[1]} 的请求")
                print(f"消息内容: {message}")
                
                # 处理列出地图请求
                if message == "LIST_MAPS":
                    print("获取地图列表...")
                    maps = get_all_maps()
                    
                    # 构建JSON响应
                    response_data = {
                        "status": "ok",
                        "count": len(maps),
                        "maps": maps
                    }
                    response = json.dumps(response_data)
                    
                    sock.sendto(response.encode('utf-8'), addr)
                    print(f"已返回 {len(maps)} 个地图信息")
                    print(f"响应长度: {len(response)} bytes\n")
                
                # 处理上传指定地图请求
                elif message.startswith("UPLOAD_MAP:"):
                    try:
                        # 提取地图标识符（可以是数字或名称）
                        map_identifier = message.split(':', 1)[1].strip()
                        print(f"上传指定地图: {map_identifier}")
                        
                        success, uploaded_files = upload_specific_map(map_identifier)
                        
                        if success:
                            response = f"OK:Map {map_identifier} uploaded ({', '.join(uploaded_files)})"
                            print(f"✓ 地图 {map_identifier} 上传完成")
                        else:
                            response = f"ERROR:Failed to upload map {map_identifier}"
                            print(f"✗ 地图 {map_identifier} 上传失败")
                        
                        sock.sendto(response.encode('utf-8'), addr)
                        print(f"已发送响应到 {addr[0]}:{addr[1]}: {response}\n")
                        
                    except IndexError as e:
                        error_msg = f"ERROR:Missing map identifier"
                        sock.sendto(error_msg.encode('utf-8'), addr)
                        print(f"✗ 缺少地图标识符: {e}\n")
                    except Exception as e:
                        error_msg = f"ERROR:{str(e)}"
                        sock.sendto(error_msg.encode('utf-8'), addr)
                        print(f"✗ 处理上传请求时出错: {e}\n")
                
                # 处理上传最新地图请求（保持兼容）
                elif message == "UPLOAD_MAP":
                    # 上传最新地图（.pgm + .yaml）
                    print(f"开始上传最新地图...")
                    success, map_number, uploaded_files = upload_latest_map()
                    
                    # 发送响应
                    if success:
                        response = f"OK:Map {map_number} uploaded successfully ({', '.join(uploaded_files)})"
                        print(f"✓ 地图上传完成")
                    else:
                        response = "ERROR:Failed to upload map"
                        print(f"✗ 地图上传失败")
                    
                    sock.sendto(response.encode('utf-8'), addr)
                    print(f"已发送响应到 {addr[0]}:{addr[1]}: {response}\n")
                    
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

def main():
    # 声明全局变量
    global FTP_SERVER_IP, FTP_PORT
    
    # 如果有命令行参数，使用旧的直接上传模式（兼容性）
    if len(sys.argv) >= 2:
        print("=== 直接上传模式 ===")
        filename = sys.argv[1]
        server_ip = sys.argv[2] if len(sys.argv) > 2 else FTP_SERVER_IP
        port = int(sys.argv[3]) if len(sys.argv) > 3 else FTP_PORT
        
        # 临时修改配置
        FTP_SERVER_IP = server_ip
        FTP_PORT = port
        
        # 判断是否需要上传配对文件
        if filename.endswith('.pgm'):
            yaml_file = filename.replace('.pgm', '.yaml')
            success = upload_map_ftp(filename)
            if success and os.path.exists(yaml_file):
                upload_map_ftp(yaml_file)
            sys.exit(0 if success else 1)
        else:
            success = upload_map_ftp(filename)
            sys.exit(0 if success else 1)
    else:
        # 启动监听服务模式
        print("=== 监听服务模式 ===")
        sys.exit(start_listener())

if __name__ == '__main__':
    main()
