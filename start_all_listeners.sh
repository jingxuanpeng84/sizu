#!/bin/bash
# 启动所有SLAM端监听服务

echo "=== 启动SLAM端监听服务 ==="
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

if [ ! -f "upload_map_ftp.py" ]; then
    echo "错误: 找不到 upload_map_ftp.py"
    exit 1
fi

# 给脚本添加执行权限
chmod +x save_map_listener.py
chmod +x upload_map_ftp.py

echo "1. 启动地图保存监听服务（端口 9999）..."
python3 save_map_listener.py &
SAVE_PID=$!
echo "   进程 ID: $SAVE_PID"

sleep 1

echo "2. 启动地图上传监听服务（端口 9998）..."
python3 upload_map_ftp.py &
UPLOAD_PID=$!
echo "   进程 ID: $UPLOAD_PID"

echo ""
echo "=== 所有监听服务已启动 ==="
echo "保存地图服务 PID: $SAVE_PID (端口 9999)"
echo "上传地图服务 PID: $UPLOAD_PID (端口 9998)"
echo ""
echo "按 Ctrl+C 停止所有服务"
echo ""

# 等待任一进程退出
wait -n

# 如果一个进程退出，杀死另一个
echo ""
echo "检测到服务退出，正在停止所有服务..."
kill $SAVE_PID 2>/dev/null
kill $UPLOAD_PID 2>/dev/null

echo "所有服务已停止"
