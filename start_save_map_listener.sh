#!/bin/bash
# 启动SLAM端地图保存监听服务

echo "=== 启动SLAM端地图保存监听服务 ==="
echo "此服务监听来自服务器端的保存地图请求"
echo "监听端口: 9999"
echo ""

# 检查 Python3 是否安装
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到 python3，请先安装"
    exit 1
fi

# 检查脚本文件是否存在
if [ ! -f "save_map_listener.py" ]; then
    echo "错误: 找不到 save_map_listener.py"
    exit 1
fi

# 检查 FTP 上传脚本是否存在
if [ ! -f "upload_map_ftp.py" ]; then
    echo "警告: 找不到 upload_map_ftp.py，将跳过 FTP 上传"
fi

# 给脚本添加执行权限
chmod +x save_map_listener.py

# 启动监听服务
python3 save_map_listener.py

# 监听服务退出后，检查退出状态
EXIT_CODE=$?

echo ""
echo "=== 监听服务已退出（退出码: $EXIT_CODE）==="

# 如果监听服务正常退出（退出码为0），则上传地图
if [ $EXIT_CODE -eq 0 ]; then
    echo ""
    echo "=== 开始上传地图到服务器端 ==="
    
    # 地图文件路径
    MAP_FILE="/home/orangepi/slam/src/FAST_LIO_GLOBAL/PCD/scans.pgm"
    
    # 检查地图文件是否存在
    if [ -f "$MAP_FILE" ]; then
        echo "找到地图文件: $MAP_FILE"
        
        # 检查 FTP 上传脚本
        if [ -f "upload_map_ftp.py" ]; then
            echo "正在上传..."
            python3 upload_map_ftp.py "$MAP_FILE" 192.168.31.70 2121
            
            UPLOAD_EXIT_CODE=$?
            if [ $UPLOAD_EXIT_CODE -eq 0 ]; then
                echo "✓ 地图上传成功"
            else
                echo "✗ 地图上传失败（退出码: $UPLOAD_EXIT_CODE）"
            fi
        else
            echo "✗ 找不到 upload_map_ftp.py，跳过上传"
        fi
    else
        echo "⚠ 地图文件不存在: $MAP_FILE"
        echo "可能保存失败，请检查日志"
    fi
else
    echo "⚠ 监听服务异常退出，跳过地图上传"
fi

echo ""
echo "=== 脚本执行完成 ==="
