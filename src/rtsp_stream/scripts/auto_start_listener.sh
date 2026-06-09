#!/bin/bash
###############################################################################
# RTSP 推流系统 - 完整自启动脚本
# 
# 功能：同时配置 MediaMTX 和监听程序的开机自启动
# 安装方法：
#   1. 修改下面的路径配置
#   2. sudo chmod +x auto_start_listener.sh
#   3. sudo ./auto_start_listener.sh install
###############################################################################

# ==================== 配置区域 ====================
# ROS 工作空间路径（请根据实际情况修改）
ROS_WORKSPACE="/home/firefly/slam_ws"

# ROS 发行版（melodic/noetic等）
ROS_DISTRO="melodic"

# 用户名（运行程序的用户）
ROS_USER="firefly"

# MediaMTX 可执行文件路径
MEDIAMTX_PATH="/home/firefly/mediamtx"

# 监听端口
LISTEN_PORT="8113"

# 推流启动命令（完整的 bash 命令）
LAUNCH_COMMAND="bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source $ROS_WORKSPACE/devel/setup.bash && roslaunch rtsp_stream map_publisher.launch'"

# 日志文件路径
LOG_DIR="/var/log/rtsp_stream"
MEDIAMTX_LOG="$LOG_DIR/mediamtx.log"
LISTENER_LOG="$LOG_DIR/listener.log"

# 监听程序可执行文件路径
LISTENER_EXECUTABLE="$ROS_WORKSPACE/devel/lib/rtsp_stream/stream_listener"

# systemd 服务名称
MEDIAMTX_SERVICE="mediamtx"
LISTENER_SERVICE="rtsp-stream-listener"
# ==================================================

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 检查是否以 root 权限运行
check_root() {
    if [ "$EUID" -ne 0 ]; then
        print_error "Please run as root (use sudo)"
        exit 1
    fi
}

# 创建日志目录
create_log_directory() {
    mkdir -p "$LOG_DIR"
    chown "$ROS_USER:$ROS_USER" "$LOG_DIR"
    print_info "Created log directory: $LOG_DIR"
}

# 创建 MediaMTX systemd 服务文件
create_mediamtx_service() {
    print_step "Creating MediaMTX service file..."
    
    cat > "/etc/systemd/system/${MEDIAMTX_SERVICE}.service" << EOF
[Unit]
Description=MediaMTX RTSP Server
After=network.target
Before=${LISTENER_SERVICE}.service

[Service]
Type=simple
User=$ROS_USER
ExecStart=$MEDIAMTX_PATH
WorkingDirectory=$(dirname $MEDIAMTX_PATH)
Restart=always
RestartSec=5

# 日志
StandardOutput=append:$MEDIAMTX_LOG
StandardError=append:$MEDIAMTX_LOG

[Install]
WantedBy=multi-user.target
EOF

    print_info "MediaMTX service file created"
}

# 创建监听程序 systemd 服务文件
create_listener_service() {
    print_step "Creating Stream Listener service file..."
    
    cat > "/etc/systemd/system/${LISTENER_SERVICE}.service" << EOF
[Unit]
Description=RTSP Stream Listener Service
After=network.target ${MEDIAMTX_SERVICE}.service
Requires=${MEDIAMTX_SERVICE}.service

[Service]
Type=simple
User=$ROS_USER
WorkingDirectory=$ROS_WORKSPACE

# 启动命令（使用独立版本，不依赖 ROS）
ExecStart=$LISTENER_EXECUTABLE -p $LISTEN_PORT -c "$LAUNCH_COMMAND" -l $LISTENER_LOG

# 重启策略
Restart=always
RestartSec=10

# 日志
StandardOutput=append:$LISTENER_LOG
StandardError=append:$LISTENER_LOG

[Install]
WantedBy=multi-user.target
EOF

    print_info "Listener service file created"
}

# 安装服务
install_service() {
    check_root
    
    print_info "=========================================="
    print_info "Installing RTSP Stream System Services"
    print_info "=========================================="
    echo ""
    
    # 检查 MediaMTX 是否存在
    if [ ! -f "$MEDIAMTX_PATH" ]; then
        print_error "MediaMTX not found: $MEDIAMTX_PATH"
        print_error "Please install MediaMTX first"
        exit 1
    fi
    
    # 检查工作空间是否存在
    if [ ! -d "$ROS_WORKSPACE" ]; then
        print_error "ROS workspace not found: $ROS_WORKSPACE"
        print_error "Please modify ROS_WORKSPACE in this script"
        exit 1
    fi
    
    # 检查监听程序是否存在
    if [ ! -f "$LISTENER_EXECUTABLE" ]; then
        print_error "Listener executable not found: $LISTENER_EXECUTABLE"
        print_error "Please run: cd $ROS_WORKSPACE && catkin_make"
        exit 1
    fi
    
    # 创建日志目录
    create_log_directory
    
    # 创建服务文件
    print_step "Step 1/5: Creating service files..."
    create_mediamtx_service
    create_listener_service
    echo ""
    
    # 重新加载 systemd
    print_step "Step 2/5: Reloading systemd daemon..."
    systemctl daemon-reload
    echo ""
    
    # 启用服务（开机自启动）
    print_step "Step 3/5: Enabling services..."
    systemctl enable "${MEDIAMTX_SERVICE}.service"
    systemctl enable "${LISTENER_SERVICE}.service"
    echo ""
    
    # 启动服务
    print_step "Step 4/5: Starting services..."
    systemctl start "${MEDIAMTX_SERVICE}.service"
    sleep 2
    systemctl start "${LISTENER_SERVICE}.service"
    sleep 2
    echo ""
    
    # 检查状态
    print_step "Step 5/5: Checking service status..."
    echo ""
    
    if systemctl is-active --quiet "${MEDIAMTX_SERVICE}.service"; then
        print_info "✓ MediaMTX service is running"
    else
        print_error "✗ MediaMTX service failed to start"
    fi
    
    if systemctl is-active --quiet "${LISTENER_SERVICE}.service"; then
        print_info "✓ Stream Listener service is running"
    else
        print_error "✗ Stream Listener service failed to start"
    fi
    
    echo ""
    print_info "=========================================="
    print_info "Installation Complete!"
    print_info "=========================================="
    echo ""
    print_info "Services:"
    print_info "  1. ${MEDIAMTX_SERVICE} - RTSP Server (Port 8554)"
    print_info "  2. ${LISTENER_SERVICE} - Stream Listener (Port ${LISTEN_PORT})"
    echo ""
    print_info "Commands:"
    print_info "  Status:  sudo systemctl status ${MEDIAMTX_SERVICE}"
    print_info "           sudo systemctl status ${LISTENER_SERVICE}"
    print_info "  Logs:    sudo journalctl -u ${MEDIAMTX_SERVICE} -f"
    print_info "           sudo journalctl -u ${LISTENER_SERVICE} -f"
    echo ""
}

# 卸载服务
uninstall_service() {
    check_root
    
    print_info "Uninstalling RTSP Stream System Services..."
    echo ""
    
    # 停止服务
    print_step "Stopping services..."
    systemctl stop "${LISTENER_SERVICE}.service" 2>/dev/null || true
    systemctl stop "${MEDIAMTX_SERVICE}.service" 2>/dev/null || true
    echo ""
    
    # 禁用服务
    print_step "Disabling services..."
    systemctl disable "${LISTENER_SERVICE}.service" 2>/dev/null || true
    systemctl disable "${MEDIAMTX_SERVICE}.service" 2>/dev/null || true
    echo ""
    
    # 删除服务文件
    print_step "Removing service files..."
    rm -f "/etc/systemd/system/${LISTENER_SERVICE}.service"
    rm -f "/etc/systemd/system/${MEDIAMTX_SERVICE}.service"
    echo ""
    
    # 重新加载 systemd
    print_step "Reloading systemd daemon..."
    systemctl daemon-reload
    echo ""
    
    print_info "Services uninstalled successfully!"
}

# 查看状态
status_service() {
    print_info "=========================================="
    print_info "Service Status"
    print_info "=========================================="
    echo ""
    
    print_info "MediaMTX Service:"
    systemctl status "${MEDIAMTX_SERVICE}.service" --no-pager
    echo ""
    
    print_info "Stream Listener Service:"
    systemctl status "${LISTENER_SERVICE}.service" --no-pager
}

# 查看日志
logs_service() {
    if [ "$2" == "mediamtx" ]; then
        print_info "Showing MediaMTX logs (Ctrl+C to exit):"
        journalctl -u "${MEDIAMTX_SERVICE}.service" -f
    elif [ "$2" == "listener" ]; then
        print_info "Showing Stream Listener logs (Ctrl+C to exit):"
        journalctl -u "${LISTENER_SERVICE}.service" -f
    else
        print_info "Showing all logs (Ctrl+C to exit):"
        journalctl -u "${MEDIAMTX_SERVICE}.service" -u "${LISTENER_SERVICE}.service" -f
    fi
}

# 重启服务
restart_service() {
    check_root
    
    print_info "Restarting services..."
    systemctl restart "${MEDIAMTX_SERVICE}.service"
    sleep 2
    systemctl restart "${LISTENER_SERVICE}.service"
    sleep 2
    
    print_info "Services restarted!"
    status_service
}

# 测试服务（不安装）
test_service() {
    print_info "Testing listener service (not installing)..."
    print_info "Press Ctrl+C to stop"
    print_info ""
    
    if [ ! -f "$LISTENER_EXECUTABLE" ]; then
        print_error "Executable not found: $LISTENER_EXECUTABLE"
        print_error "Please run: cd $ROS_WORKSPACE && catkin_make"
        exit 1
    fi
    
    $LISTENER_EXECUTABLE -p $LISTEN_PORT -c "$LAUNCH_COMMAND"
}

# 显示帮助
show_help() {
    echo "RTSP Stream System - Auto Start Script"
    echo ""
    echo "Usage: sudo $0 [command]"
    echo ""
    echo "Commands:"
    echo "  install     Install and start both services (enable auto-start)"
    echo "  uninstall   Stop and remove both services"
    echo "  status      Show services status"
    echo "  logs        Show services logs (real-time)"
    echo "              logs mediamtx  - Show only MediaMTX logs"
    echo "              logs listener  - Show only Listener logs"
    echo "  restart     Restart both services"
    echo "  test        Test run listener without installing"
    echo "  help        Show this help message"
    echo ""
    echo "Configuration:"
    echo "  ROS Workspace:     $ROS_WORKSPACE"
    echo "  ROS Distro:        $ROS_DISTRO"
    echo "  User:              $ROS_USER"
    echo "  MediaMTX Path:     $MEDIAMTX_PATH"
    echo "  Listener Port:     $LISTEN_PORT"
    echo "  MediaMTX Service:  $MEDIAMTX_SERVICE"
    echo "  Listener Service:  $LISTENER_SERVICE"
    echo ""
}

# 主程序
case "$1" in
    install)
        install_service
        ;;
    uninstall)
        uninstall_service
        ;;
    status)
        status_service
        ;;
    logs)
        logs_service "$@"
        ;;
    restart)
        restart_service
        ;;
    test)
        test_service
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
