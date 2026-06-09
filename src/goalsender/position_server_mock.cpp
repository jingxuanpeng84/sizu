#include "position_server_newdevice.hpp"
#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <signal.h>
#include <cmath>

/**
 * @file position_server_mock.cpp
 * @brief 位置服务端 - 模拟数据版本（用于测试）
 * 
 * 该版本不依赖真实ROS话题，而是生成模拟的位置数据
 * 用于测试网络通信和协议功能
 * 
 * 帧类型定义（与 httpcopy 兼容）：
 * - 0: REQUEST  - 请求帧
 * - 1: RESPONSE - 响应帧（数据上报）
 * - 2: COMMAND  - 控制命令
 * - 3: INQUIRY  - 状态查询
 * - 4: BACK     - 状态反馈
 */

// ==================== 模拟数据生成器类 ====================
class PositionServerMock : public device {
private:
    // 网络配置
    std::string myIp;
    unsigned short myPort;
    std::string httpServerIp;
    unsigned short httpServerPort;
    
    // 位置数据
    POSITION::position_data currentPositionData;
    std::mutex positionDataMutex;
    std::atomic<unsigned long> currentSeq;
    
    // UDP通信器
    udpTool* communicator;
    
    // 模拟数据生成线程
    std::thread mockDataThread;
    std::atomic<bool> mockDataRunning;
    
    // 模拟数据生成函数
    void generateMockData() {
        double time = 0.0;
        int iteration = 0;
        
        while (mockDataRunning) {
            {
                std::lock_guard<std::mutex> lock(positionDataMutex);
                
                // 模拟机器人在圆形轨迹上移动
                double radius = 3.0;  // 3米半径
                currentPositionData.x = radius * cos(time);
                currentPositionData.y = radius * sin(time);
                currentPositionData.yaw = time + M_PI / 2;  // 切线方向
                
                time += 0.05;  // 每次增加0.05弧度
                if (time > 2 * M_PI) time -= 2 * M_PI;
            }
            
            // 定期打印模拟数据状态
            if (++iteration % 20 == 0) {
                std::cout << "[MockPosition] 模拟数据更新 - 位置: (" 
                          << currentPositionData.x << ", " << currentPositionData.y 
                          << "), 航向: " << (currentPositionData.yaw * 180.0 / M_PI) << " deg" << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz更新率
        }
    }
    
    // 静态回调函数
    static void mockDataCallback(device* dev) {
        auto data = dev->frontData();
        if (data.first == nullptr) return;
        
        struct frame* frameData = (struct frame*)data.first;
        
        if (frameData->type == FRAME_TYPE_HEARTBEAT) {
            struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
            if (hbFrame->type == HEARTBEAT_COMMAND_RESTART) {
                std::cout << "[MockPosition] 收到重启命令" << std::endl;
                dev->ResetModule();
            }
        }
        
        dev->popData();
    }
    
    static std::pair<std::string, int> mockResetCallback(device* dev) {
        std::cout << "[MockPosition] 执行重启..." << std::endl;
        
        PositionServerMock* mockServer = static_cast<PositionServerMock*>(dev);
        if (mockServer) {
            mockServer->currentSeq = 0;
            std::lock_guard<std::mutex> lock(mockServer->positionDataMutex);
            memset(&mockServer->currentPositionData, 0, sizeof(mockServer->currentPositionData));
        }
        
        return std::make_pair("模拟位置服务重启成功", 0);
    }
    
    static std::pair<std::string, int> mockCheckCallback(device* dev) {
        dev->feedWatchdog();
        return std::make_pair("模拟位置服务正常", 0);
    }
    
    // UDP接收回调
    static void udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
        // 原始数据调试
        std::cout << "[MockPosition] UDP收到数据包: size=" << size 
                  << " from " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
        
        if (!buf || size <= 0) {
            std::cout << "[MockPosition] 数据包为空或大小无效" << std::endl;
            return;
        }
        
        // 打印前24字节的原始数据（frame头部）
        if (size >= 24) {
            unsigned char* rawBuf = (unsigned char*)buf;
            std::cout << "[MockPosition] Frame头部(前24字节): ";
            for (int i = 0; i < std::min(24, size); i++) {
                printf("%02X ", rawBuf[i]);
            }
            std::cout << std::endl;
        }
        
        struct frame* frameData = (struct frame*)buf;
        std::cout << "[MockPosition] Frame解析: source=" << frameData->source 
                  << " dest=" << frameData->dest 
                  << " type=" << frameData->type 
                  << " len_data=" << frameData->len_data << std::endl;
        
        device* dev = (device*)net->getPrivateData();
        PositionServerMock* mockServer = static_cast<PositionServerMock*>(dev);
        
        if (!mockServer) return;
        
        // 处理心跳帧
        if (frameData->type == FRAME_TYPE_HEARTBEAT) {
            struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
            if (hbFrame->type == HEARTBEAT_COMMAND_RESTART && dev) {
                dev->pushData((char*)buf, size);
            }
            return;
        }
        
        // 处理 INQUIRY 帧（来自 request_slam_pose 的临时 socket）
        // INQUIRY 类型在 httpcopy 中定义为 3
        if (frameData->type == 3 && frameData->source == HTTPSERVER_OBJ) {
            std::cout << "[MockPosition] 收到 INQUIRY 请求 from " 
                      << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
            
            // 直接响应到请求来源地址
            mockServer->handleInquiryRequest(addr);
            return;
        }
        
        // 处理数据请求帧
        if (frameData->type == FRAME_TYPE_DATA && frameData->source == HTTPSERVER_OBJ) {
            if (frameData->len_data >= sizeof(POSITION::requestFrameStruct)) {
                POSITION::requestFrameStruct* request = (POSITION::requestFrameStruct*)frameData->data;
                
                std::cout << "[MockPosition] 收到HTTPSERVER请求，seq: " << request->seq 
                          << " from " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
                
                // 发送模拟数据
                mockServer->handleDataRequest(*request);
            }
            
            if (dev) dev->pushData((char*)buf, size);
        }
    }
    
    // 处理 INQUIRY 请求（响应到请求来源）
    void handleInquiryRequest(const sockaddr_in& clientAddr) {
        POSITION::position_data currentData;
        {
            std::lock_guard<std::mutex> lock(positionDataMutex);
            currentData = currentPositionData;
        }
        
        // 构造响应帧
        POSITION::receiveFrameStruct response;
        response.frame_type = 1;
        response.seq = 0;
        response.data = currentData;
        
        // 构造完整的 frame 包
        size_t totalSize = sizeof(struct frame) + sizeof(response);
        char* sendBuf = new char[totalSize];
        
        struct frame* frameData = (struct frame*)sendBuf;
        frameData->source = LADDAR_OBJ;
        frameData->dest = HTTPSERVER_OBJ;
        frameData->type = 1;  // RESPONSE 类型（在 httpcopy 中定义为 1）
        frameData->len_data = sizeof(response);
        
        memcpy(frameData->data, &response, sizeof(response));
        
        // 直接发送到请求来源地址
        if (this->communicator) {
            ipPort clientIp{inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port)};
            this->communicator->sendData(sendBuf, totalSize, clientIp);
            
            std::cout << "[MockPosition] 已响应 INQUIRY 到 " 
                      << clientIp.ip << ":" << clientIp.port 
                      << " 位置: (" << currentData.x << ", " << currentData.y << ")" << std::endl;
        }
        
        delete[] sendBuf;
    }
    
    void handleDataRequest(const POSITION::requestFrameStruct& request) {
        POSITION::position_data currentData;
        {
            std::lock_guard<std::mutex> lock(positionDataMutex);
            currentData = currentPositionData;
        }
        
        // 构造响应帧（匹配客户端期望）
        POSITION::receiveFrameStruct response;
        response.frame_type = 1;
        response.seq = request.seq;
        response.data = currentData;
        
        // 直接使用 communicator 发送，frameType=4 (FRAME_TYPE_RESPONSE)
        if (this->communicator) {
            this->communicator->sendDataByMap(&response, sizeof(response), HTTPSERVER_OBJ, 4);
        }
        
        static int sendCount = 0;
        sendCount++;
        if (sendCount % 50 == 0) {
            std::cout << "[MockPosition] 已发送 " << sendCount << " 帧模拟数据到 HTTPSERVER_OBJ" << std::endl;
        }
    }

public:
    PositionServerMock(const char *myIp_, unsigned short myPort_,
                       const char *httpServerIp_, unsigned short httpServerPort_)
        : device("MockPositionServer", nullptr, DeviceCallbackTable(
            PositionServerMock::mockDataCallback,
            PositionServerMock::mockResetCallback,
            PositionServerMock::mockCheckCallback
        ), LADDAR_OBJ)
    {
        this->myIp = std::string(myIp_);
        this->myPort = myPort_;
        this->httpServerIp = std::string(httpServerIp_);
        this->httpServerPort = httpServerPort_;
        this->currentSeq = 0;
        this->communicator = nullptr;
        this->mockDataRunning = false;
        
        memset(&this->currentPositionData, 0, sizeof(this->currentPositionData));
        
        std::cout << "[MockPosition] 模拟位置服务端已创建" << std::endl;
    }
    
    ~PositionServerMock() {
        endWork();
        watingDeviceEnding();
    }
    
    bool startWork() {
        // 创建 UDP 通信器
        struct ipPort localAddr = {this->myIp, this->myPort};
        this->communicator = new udpTool(localAddr, MODULE_TYPE::LADDAR_OBJ);

        if (this->communicator->createNet(FRAME_MAX_SIZE) != SUCCESS) {
            std::cerr << "[MockPosition] sock error" << std::endl;
            delete this->communicator;
            this->communicator = nullptr;
            return false;
        }
        
        // 注册HTTPSERVER_OBJ地址映射
        this->communicator->registerDestIpMap(this->httpServerIp, this->httpServerPort, HTTPSERVER_OBJ);
        this->communicator->setPrivateData(this);
        
        // 启动接收线程
        if (this->communicator->startRecvThread(PositionServerMock::udpRecvCallback) != SUCCESS) {
            std::cerr << "[MockPosition] thread start error" << std::endl;
            delete this->communicator;
            this->communicator = nullptr;
            return false;
        }
        
        // 启动心跳
        this->startHeartbeat();
        
        // 启动模拟数据生成线程
        mockDataRunning = true;
        mockDataThread = std::thread(&PositionServerMock::generateMockData, this);
        
        std::cout << "[MockPosition] 模拟位置服务启动成功!" << std::endl;
        std::cout << "  - 监听端口: " << this->myPort << std::endl;
        std::cout << "  - 目标HTTPSERVER: " << this->httpServerIp << ":" << this->httpServerPort << std::endl;
        std::cout << "  - 模拟数据生成器已启动 (10Hz)" << std::endl;
        
        return true;
    }
    
    bool endWork() {
        // 停止模拟数据生成
        mockDataRunning = false;
        if (mockDataThread.joinable()) {
            mockDataThread.join();
        }
        
        // 停止心跳
        this->stopHeartbeat();
        
        // 停止接收线程
        if (this->communicator) {
            this->communicator->endRecvThread();
        }
        
        return true;
    }
    
    void watingDeviceEnding() {
        if (this->communicator) {
            this->communicator->destroryNet();
            delete this->communicator;
            this->communicator = nullptr;
        }
        
        std::cout << "[MockPosition] 模拟位置服务结束" << std::endl;
    }
};

// ==================== 主函数 ====================

PositionServerMock* g_mock_position_server = nullptr;

void signalHandler(int signum) {
    std::cout << "\n[MockPosition] 收到中断信号 (" << signum << ")，正在关闭..." << std::endl;
    if (g_mock_position_server) {
        g_mock_position_server->endWork();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "position_server_mock");
    ros::NodeHandle nh("~");
    
    std::cout << "========================================" << std::endl;
    std::cout << "  位置服务端 - 模拟数据测试版本" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 从ROS参数服务器读取配置
    std::string position_ip, httpserver_ip;
    int position_port, httpserver_port;
    
    nh.param<std::string>("position_ip", position_ip, "192.168.1.100");
    nh.param<int>("position_port", position_port, 7070);
    nh.param<std::string>("httpserver_ip", httpserver_ip, "192.168.1.200");
    nh.param<int>("httpserver_port", httpserver_port, 8888);
    
    std::cout << "\n=== 配置信息 ===" << std::endl;
    std::cout << "位置服务端地址: " << position_ip << ":" << position_port << std::endl;
    std::cout << "HTTP服务器地址: " << httpserver_ip << ":" << httpserver_port << std::endl;
    std::cout << "数据模式: 模拟数据（圆形轨迹运动）" << std::endl;
    
    // 创建模拟位置服务端
    g_mock_position_server = new PositionServerMock(
        position_ip.c_str(),
        static_cast<unsigned short>(position_port),
        httpserver_ip.c_str(),
        static_cast<unsigned short>(httpserver_port)
    );
    
    // 启动服务
    if (!g_mock_position_server->startWork()) {
        std::cerr << "[MockPosition] 启动失败!" << std::endl;
        delete g_mock_position_server;
        return -1;
    }
    
    std::cout << "\n[MockPosition] 服务已启动，等待HTTPSERVER请求..." << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    
    // 使用ROS事件循环
    ros::Rate rate(10);  // 10Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // 清理
    std::cout << "\n[MockPosition] 正在关闭服务..." << std::endl;
    g_mock_position_server->endWork();
    g_mock_position_server->watingDeviceEnding();
    delete g_mock_position_server;
    g_mock_position_server = nullptr;
    
    std::cout << "[MockPosition] 服务已关闭" << std::endl;
    return 0;
}
