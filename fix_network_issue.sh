#!/bin/bash
# 网络问题诊断和修复脚本
# 用于解决 UDP 请求来源错误的问题

echo "=========================================="
echo "  UDP 通信问题诊断和修复工具"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 步骤 1: 检查当前机器的 IP 地址
echo "步骤 1: 检查当前机器的网络配置"
echo "----------------------------------------"
echo "当前机器的 IP 地址："
ip addr show | grep "inet " | grep -v "127.0.0.1"
echo ""

CURRENT_IP=$(ip addr show | grep "inet " | grep -v "127.0.0.1" | head -1 | awk '{print $2}' | cut -d'/' -f1)
echo "检测到主 IP: $CURRENT_IP"
echo ""

# 步骤 2: 检查是否有 configserver 进程在运行
echo "步骤 2: 检查 configserver 进程"
echo "----------------------------------------"
CONFIGSERVER_PIDS=$(pgrep -f configserver)

if [ -z "$CONFIGSERVER_PIDS" ]; then
    echo -e "${GREEN}✓ 当前没有 configserver 进程运行${NC}"
else
    echo -e "${YELLOW}⚠ 发现 configserver 进程：${NC}"
    ps aux | grep configserver | grep -v grep
    echo ""
    
    # 如果当前机器是 192.168.110.180，提示停止进程
    if [[ "$CURRENT_IP" == "192.168.110.180" ]]; then
        echo -e "${RED}✗ 警告：当前机器是 192.168.110.180（旧服务器）${NC}"
        echo -e "${RED}  这些进程应该被停止！${NC}"
        echo ""
        read -p "是否停止所有 configserver 进程？(y/n) " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            echo "正在停止 configserver 进程..."
            pkill -9 -f configserver
            sleep 1
            
            # 验证是否停止
            REMAINING=$(pgrep -f configserver)
            if [ -z "$REMAINING" ]; then
                echo -e "${GREEN}✓ 所有 configserver 进程已停止${NC}"
            else
                echo -e "${RED}✗ 部分进程仍在运行，请手动停止：${NC}"
                ps aux | grep configserver | grep -v grep
            fi
        fi
    elif [[ "$CURRENT_IP" == "192.168.110.151" ]]; then
        echo -e "${GREEN}✓ 当前机器是 192.168.110.151（新服务器）${NC}"
        echo -e "${GREEN}  configserver 进程正常运行${NC}"
    else
        echo -e "${YELLOW}⚠ 当前机器 IP ($CURRENT_IP) 不是预期的服务器地址${NC}"
    fi
fi
echo ""

# 步骤 3: 检查端口占用情况
echo "步骤 3: 检查关键端口占用"
echo "----------------------------------------"

check_port() {
    local port=$1
    local name=$2
    local result=$(netstat -tuln 2>/dev/null | grep ":$port " || ss -tuln 2>/dev/null | grep ":$port ")
    
    if [ -z "$result" ]; then
        echo -e "${YELLOW}⚠ 端口 $port ($name) 未被占用${NC}"
    else
        echo -e "${GREEN}✓ 端口 $port ($name) 正在监听${NC}"
        echo "  $result"
    fi
}

check_port 8080 "HTTP服务器"
check_port 2121 "FTP服务器"
check_port 9998 "地图上传监听"
check_port 9999 "地图保存监听"
echo ""

# 步骤 4: 测试 UDP 连通性
echo "步骤 4: 测试 UDP 连通性"
echo "----------------------------------------"

if [[ "$CURRENT_IP" == "192.168.110.151" ]]; then
    echo "测试到 SLAM端 (192.168.110.205:9998) 的 UDP 连接..."
    
    # 检查 nc 命令是否可用
    if command -v nc &> /dev/null; then
        echo "TEST_CONNECTION" | timeout 2 nc -u 192.168.110.205 9998 2>/dev/null
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ UDP 连接测试成功${NC}"
        else
            echo -e "${YELLOW}⚠ UDP 连接测试超时（这是正常的，因为没有响应）${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ nc 命令不可用，跳过连接测试${NC}"
        echo "  可以安装: sudo apt-get install netcat"
    fi
elif [[ "$CURRENT_IP" == "192.168.110.205" ]]; then
    echo "当前是 SLAM端，可以使用 tcpdump 监听 UDP 请求："
    echo "  sudo tcpdump -i any -n udp port 9998"
else
    echo "当前机器不是服务器端或 SLAM端，跳过连接测试"
fi
echo ""

# 步骤 5: 提供操作建议
echo "=========================================="
echo "  操作建议"
echo "=========================================="
echo ""

if [[ "$CURRENT_IP" == "192.168.110.180" ]]; then
    echo -e "${RED}当前机器：192.168.110.180（旧服务器）${NC}"
    echo ""
    echo "建议操作："
    echo "1. 停止所有 configserver 进程（如果还在运行）"
    echo "   pkill -9 -f configserver"
    echo ""
    echo "2. 不要在这台机器上启动新的 configserver"
    echo ""
    echo "3. 确保浏览器访问的是新服务器："
    echo "   http://192.168.110.151:8080"
    echo ""
    
elif [[ "$CURRENT_IP" == "192.168.110.151" ]]; then
    echo -e "${GREEN}当前机器：192.168.110.151（新服务器）${NC}"
    echo ""
    echo "建议操作："
    
    if [ -z "$CONFIGSERVER_PIDS" ]; then
        echo "1. 启动 configserver："
        echo "   cd ~/Desktop/work_station/httpcopy_0421/build"
        echo "   ./configserver"
    else
        echo "1. configserver 已在运行，无需操作"
    fi
    echo ""
    echo "2. 确保浏览器访问："
    echo "   http://192.168.110.151:8080"
    echo ""
    echo "3. 在 SLAM端 (192.168.110.205) 监听 UDP 请求："
    echo "   sudo tcpdump -i any -n udp port 9998"
    echo "   应该看到来自 192.168.110.151 的请求"
    echo ""
    
elif [[ "$CURRENT_IP" == "192.168.110.205" ]]; then
    echo -e "${GREEN}当前机器：192.168.110.205（SLAM端）${NC}"
    echo ""
    echo "建议操作："
    echo "1. 确保监听服务正在运行："
    echo "   bash start_all_listeners.sh"
    echo ""
    echo "2. 监听 UDP 请求来源："
    echo "   sudo tcpdump -i any -n udp port 9998"
    echo "   应该看到来自 192.168.110.151 的请求（不是 .180）"
    echo ""
    echo "3. 查看 upload_map_ftp.py 的日志输出"
    echo "   应该显示：收到来自 192.168.110.151:xxxxx 的请求"
    echo ""
else
    echo -e "${YELLOW}当前机器 IP: $CURRENT_IP${NC}"
    echo ""
    echo "这不是预期的服务器或 SLAM端地址。"
    echo "预期地址："
    echo "  - 服务器端（新）: 192.168.110.151"
    echo "  - 服务器端（旧）: 192.168.110.180（应停止）"
    echo "  - SLAM端: 192.168.110.205"
fi

echo ""
echo "=========================================="
echo "  诊断完成"
echo "=========================================="
