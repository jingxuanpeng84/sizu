#!/bin/bash

# RTSP 地图推流启动脚本
# 功能：启动 MediaMTX 服务器和地图推流节点

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MEDIAMTX_DIR="$HOME/mediamtx"
MEDIAMTX_BIN="$MEDIAMTX_DIR/mediamtx"
TUILIU_SCRIPT="$SCRIPT_DIR/tuiliu.py"

echo "=== RTSP 地图推流启动脚本 ==="

# 检查 MediaMTX 是否已安装
if [ ! -f "$MEDIAMTX_BIN" ]; then
    echo "MediaMTX 未安装，开始下载..."
    mkdir -p "$MEDIAMTX_DIR"
    cd "$MEDIAMTX_DIR"
    
    # 下载 ARM64 版本
    wget https://ghproxy.com/https://github.com/bluenviron/mediamtx/releases/download/v1.5.0/mediamtx_v1.5.0_linux_arm64v8.tar.gz
    
    if [ $? -ne 0 ]; then
        echo "错误：下载失败，请检查网络连接"
        exit 1
    fi
    
    tar -xzf mediamtx_v1.5.0_linux_arm64v8.tar.gz
    chmod +x mediamtx
    echo "MediaMTX 安装完成"
fi

# 检查 MediaMTX 是否已在运行
if pgrep -x "mediamtx" > /dev/null; then
    echo "MediaMTX 已在运行"
else
    echo "启动 MediaMTX 服务器..."
    cd "$MEDIAMTX_DIR"
    nohup ./mediamtx > mediamtx.log 2>&1 &
    MEDIAMTX_PID=$!
    echo "MediaMTX 已启动 (PID: $MEDIAMTX_PID)"
    
    # 等待服务启动
    sleep 2
    
    # 验证端口
    if netstat -tuln | grep -q 8554; then
        echo "✓ RTSP 服务器已就绪 (端口 8554)"
    else
        echo "✗ 警告：8554 端口未监听，请检查 MediaMTX 日志"
    fi
fi

# 检查 tuiliu.py 是否存在
if [ ! -f "$TUILIU_SCRIPT" ]; then
    echo "错误：找不到 tuiliu.py 脚本: $TUILIU_SCRIPT"
    exit 1
fi

# 启动地图推流节点
echo "启动地图推流节点..."
cd "$SCRIPT_DIR"
python3 tuiliu.py

# 清理函数
cleanup() {
    echo ""
    echo "正在停止服务..."
    pkill -f "tuiliu.py"
    # 注意：不自动停止 MediaMTX，因为可能有其他流在使用
    echo "地图推流已停止"
    echo "如需停止 MediaMTX，请运行: pkill mediamtx"
}

trap cleanup EXIT INT TERM

wait
