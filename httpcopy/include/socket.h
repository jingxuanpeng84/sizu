#ifndef SOCKET_H
#define SOCKET_H

#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include "network/frame.hpp"
#include "network/moduleObj.hpp"

// 网络通讯相关常量
#define UDP_PORT 8111
#define HTTP_PORT 8080
#define SERVER_IP "192.168.31.70"

// SLAM 位置数据
struct SlamPose {
    double x;
    double y;
    double yaw;
    int px;        // 像素坐标X
    int py;        // 像素坐标Y
    int map_id;    // 地图编号
    bool valid;
};

// 前向声明
struct FrameBuilder;

// 全局函数声明
int setup_udp_socket(int port);
void sendCommand(int sockfd, const FrameBuilder& builder, const char* ip, uint16_t port);

// 向 SLAM 请求当前位置（同步，超时 timeout_ms 毫秒）
SlamPose request_slam_pose(const char* slam_ip, uint16_t slam_port, int timeout_ms = 2000);

// 向 SLAM 请求地图文件（同步，超时 timeout_ms 毫秒）
// 返回地图数据，如果失败返回空vector
std::vector<char> request_slam_map(const char* slam_ip, uint16_t slam_port, int timeout_ms = 5000);

#endif // SOCKET_H
