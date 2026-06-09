#!/bin/bash

echo "=========================================="
echo "网络诊断工具"
echo "=========================================="
echo ""

echo "1. 本机IP地址:"
echo "---"
ip addr show | grep "inet " | grep -v "127.0.0.1"
echo ""

echo "2. 检查8111端口是否被监听:"
echo "---"
netstat -ulnp 2>/dev/null | grep 8111 || ss -ulnp 2>/dev/null | grep 8111
echo ""

echo "3. 测试到192.168.31.126的连接:"
echo "---"
ping -c 3 192.168.31.126
echo ""

echo "4. 检查防火墙状态:"
echo "---"
sudo ufw status 2>/dev/null || echo "ufw未安装或未启用"
echo ""

echo "=========================================="
echo "诊断建议:"
echo "=========================================="
echo ""
echo "如果没有收到数据，请检查："
echo "1. send.position.cpp 是否在 192.168.31.126 上运行"
echo "2. send.position.cpp 的 target_ip 是否配置为本机IP"
echo "3. send.position.cpp 是否有缓存的目标点数据"
echo ""
echo "在 192.168.31.126 上运行以下命令查看 send.position.cpp 的配置:"
echo "  rosnode info /goal_sender_node"
echo "  rosparam get /goal_sender_node/target_ip"
echo "  rosparam get /goal_sender_node/target_port"
echo ""
