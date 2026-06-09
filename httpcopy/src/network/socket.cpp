#include "socket.h"
#include "data_type.h"
#include <iostream>
#include <mutex>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>
#include "network/tcpUdp.hpp"

// SLAM 位置响应帧中 data 段的布局
// frame.data = [ mydata(16B) | SlamPosePayload ]
// 注意：这个结构体必须与 SLAM 端的 POSITION::position_data 结构体匹配
struct SlamPosePayload {
    double x;
    double y;
    double yaw;
    int px;      // 像素坐标X
    int py;      // 像素坐标Y
    int map_id;  // 地图编号
};

namespace {
    std::unique_ptr<udpTool> g_udp;
    std::mutex g_udp_mutex;
}

int setup_udp_socket(int port) {
    std::lock_guard<std::mutex> lk(g_udp_mutex);
    ipPort selfIp{"", static_cast<unsigned short>(port)};
    g_udp = std::make_unique<udpTool>(selfIp, MODULE_TYPE::HTTPSERVER_OBJ);
    if (g_udp->createNet(FRAME_MAX_SIZE) != ERROR_NUM::SUCCESS) {
        std::cerr << "Failed to initialize UDP module on port " << port << std::endl;
        g_udp.reset();
        return -1;
    }
    return 0;
}

void sendCommand(int, const FrameBuilder& builder, const char* ip, uint16_t port) {
    std::lock_guard<std::mutex> lk(g_udp_mutex);
    if (!g_udp) {
        std::cerr << "UDP module not initialized, cannot send command." << std::endl;
        return;
    }

    std::vector<char> buffer;
    builder.build_to_buffer(buffer);
    ipPort dest{std::string(ip), port};
    g_udp->sendData(buffer.data(), buffer.size(), dest);
}

// 向 SLAM 发送位置查询请求，同步等待响应
SlamPose request_slam_pose(const char* slam_ip, uint16_t slam_port, int timeout_ms) {
    SlamPose result{0.0, 0.0, 0.0, 0, 0, 1, false};

    // 创建临时 UDP socket 用于本次请求/响应
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "request_slam_pose: socket create failed" << std::endl;
        return result;
    }

    // 绑定随机本地端口
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port = 0;
    bind(fd, (sockaddr*)&local, sizeof(local));

    // 构造 INQUIRY 帧
    FrameBuilder builder{};
    builder.source = HTTP_SERVER;      // = 2 (HTTPSERVER_OBJ)
    builder.dest   = LADDAR_SYSTEM;    // = 4 (LADDAR_OBJ)
    builder.type   = FRAME_TYPE::INQUIRY;
    builder.subObj = SUBOBJ_ROBOT;
    builder.data   = {};

    std::vector<char> buf;
    builder.build_to_buffer(buf);

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(slam_port);
    inet_pton(AF_INET, slam_ip, &dest.sin_addr);

    sendto(fd, buf.data(), buf.size(), 0, (sockaddr*)&dest, sizeof(dest));

    // 等待响应
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};

    if (select(fd + 1, &fds, nullptr, nullptr, &tv) > 0) {
        char rbuf[512];
        sockaddr_in from{};
        socklen_t fromlen = sizeof(from);
        int n = recvfrom(fd, rbuf, sizeof(rbuf), 0, (sockaddr*)&from, &fromlen);
        
        // SLAM端发送的结构: frame + receiveFrameStruct
        // receiveFrameStruct = frame_type(8B) + seq(8B) + position_data
        const int header_offset = sizeof(frame) + sizeof(unsigned long) * 2;  // frame + frame_type + seq
        
        if (n >= (int)(header_offset + sizeof(SlamPosePayload))) {
            const frame* pf = reinterpret_cast<const frame*>(rbuf);
            
            // 跳过 frame_type 和 seq，直接读取 position_data
            const SlamPosePayload* pose =
                reinterpret_cast<const SlamPosePayload*>(rbuf + header_offset);
            
            result.x   = pose->x;
            result.y   = pose->y;
            result.yaw = pose->yaw;
            result.px  = pose->px;
            result.py  = pose->py;
            result.map_id = pose->map_id;  // 读取地图编号
            result.valid = true;
            
            std::cout << "收到SLAM位置响应: 世界坐标(" << result.x << ", " << result.y 
                      << "), 像素坐标(" << result.px << ", " << result.py 
                      << "), 地图编号: " << result.map_id << std::endl;
        } else {
            std::cerr << "request_slam_pose: 数据包大小不足 (收到 " << n 
                      << " 字节, 期望至少 " << (header_offset + sizeof(SlamPosePayload)) << " 字节)" << std::endl;
        }
    } else {
        std::cerr << "request_slam_pose: timeout waiting for SLAM response" << std::endl;
    }

    close(fd);
    return result;
}

// 向 SLAM 请求地图文件
std::vector<char> request_slam_map(const char* slam_ip, uint16_t slam_port, int timeout_ms) {
    std::vector<char> map_data;

    // 创建临时 UDP socket
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "request_slam_map: socket create failed" << std::endl;
        return map_data;
    }

    // 绑定随机本地端口
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = INADDR_ANY;
    local.sin_port = 0;
    bind(fd, (sockaddr*)&local, sizeof(local));

    // 构造 INQUIRY 帧请求地图
    FrameBuilder builder{};
    builder.source = HTTP_SERVER;      // = 2 (HTTPSERVER_OBJ)
    builder.dest   = LADDAR_SYSTEM;    // = 4 (LADDAR_OBJ)
    builder.type   = FRAME_TYPE::INQUIRY;
    builder.subObj = SUBOBJ_MAP;       // 请求地图数据
    builder.data   = {};

    std::vector<char> buf;
    builder.build_to_buffer(buf);

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port   = htons(slam_port);
    inet_pton(AF_INET, slam_ip, &dest.sin_addr);

    std::cout << "向 SLAM 请求地图: " << slam_ip << ":" << slam_port << std::endl;
    sendto(fd, buf.data(), buf.size(), 0, (sockaddr*)&dest, sizeof(dest));

    // 等待响应（可能需要接收多个包）
    fd_set fds;
    timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    
    // 接收缓冲区
    const int MAX_PACKET_SIZE = 65536;
    char rbuf[MAX_PACKET_SIZE];
    
    while (true) {
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        
        int ret = select(fd + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            if (map_data.empty()) {
                std::cerr << "request_slam_map: timeout waiting for SLAM map response" << std::endl;
            }
            break;
        }
        
        sockaddr_in from{};
        socklen_t fromlen = sizeof(from);
        int n = recvfrom(fd, rbuf, sizeof(rbuf), 0, (sockaddr*)&from, &fromlen);
        
        if (n > 0) {
            // 检查是否是有效的frame
            if (n >= (int)sizeof(frame)) {
                const frame* pf = reinterpret_cast<const frame*>(rbuf);
                
                // 检查是否是地图数据响应 (RESPONSE = 1)
                if (pf->type == 1) {  // RESPONSE 类型
                    // 检查 mydata 中的 sub_obj 字段
                    if (n >= (int)(sizeof(frame) + sizeof(mydata))) {
                        const mydata* md = reinterpret_cast<const mydata*>(pf->data);
                        
                        // 检查是否是地图数据 (SUBOBJ_MAP = 1)
                        if (md->sub_obj == SUBOBJ_MAP) {
                            // 提取地图数据（跳过frame头和mydata）
                            int data_offset = sizeof(frame) + sizeof(mydata);
                            if (n > data_offset) {
                                int data_len = n - data_offset;
                                map_data.insert(map_data.end(), 
                                               rbuf + data_offset, 
                                               rbuf + data_offset + data_len);
                                std::cout << "接收到地图数据包: " << data_len << " bytes, 总计: " 
                                          << map_data.size() << " bytes" << std::endl;
                            }
                            
                            // 检查是否是最后一个包
                            if (md->eof == 1 || map_data.size() > 100000) {  // eof标志或数据足够大
                                std::cout << "地图接收完成，总大小: " << map_data.size() << " bytes" << std::endl;
                                break;
                            }
                        }
                    }
                }
            }
        }
        
        // 重置超时时间为较短的等待时间，用于接收后续包
        tv.tv_sec = 1;
        tv.tv_usec = 0;
    }

    close(fd);
    return map_data;
}
