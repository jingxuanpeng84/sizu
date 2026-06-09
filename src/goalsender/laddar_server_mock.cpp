#include "include/laddar_server_newdevice.hpp"
#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include <signal.h>
#include <cmath>

/**
 * @file laddar_server_mock.cpp
 * @brief 雷达服务端 - 模拟数据版本（用于测试）
 * 
 * 该版本不依赖真实ROS话题，而是生成模拟的激光雷达和里程计数据
 * 用于测试网络通信和协议功能
 */

// ==================== 模拟数据生成器类 ====================
class LaddarServerMock : public device {
private:
    // 网络配置
    std::string myIp;
    unsigned short myPort;
    std::string robotIp;
    unsigned short robotPort;
    
    // 雷达数据
    LADDAR::laddar_data currentLaddarData;
    std::mutex laddarDataMutex;
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
                std::lock_guard<std::mutex> lock(laddarDataMutex);
                
                // 模拟机器人在圆形轨迹上移动
                double radius = 2.0;  // 2米半径
                currentLaddarData.x = radius * cos(time);
                currentLaddarData.y = radius * sin(time);
                currentLaddarData.yaw = time + M_PI / 2;  // 切线方向
                
                // 模拟激光雷达数据 - 生成一个简单的环境
                for (int i = 0; i < LINE_NUM; i++) {
                    double angle = (i * M_PI / 180.0);  // 转换为弧度
                    
                    // 模拟一个方形房间，边长10米
                    double dist_x = 5.0 / fabs(cos(angle + currentLaddarData.yaw));
                    double dist_y = 5.0 / fabs(sin(angle + currentLaddarData.yaw));
                    double dist = std::min(dist_x, dist_y);
                    
                    // 添加一些噪声
                    dist += (rand() % 100 - 50) / 1000.0;  // ±5cm噪声
                    
                    // 限制范围
                    currentLaddarData.dist[i] = std::max(0.1, std::min(dist, 30.0));
                }
                
                time += 0.05;  // 每次增加0.05弧度
                if (time > 2 * M_PI) time -= 2 * M_PI;
            }
            
            // 定期打印模拟数据状态
            if (++iteration % 20 == 0) {
                std::cout << "[MockLaddar] 模拟数据更新 - 位置: (" 
                          << currentLaddarData.x << ", " << currentLaddarData.y 
                          << "), 航向: " << (currentLaddarData.yaw * 180.0 / M_PI) << " deg" << std::endl;
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
                std::cout << "[MockLaddar] 收到重启命令" << std::endl;
                dev->ResetModule();
            }
        }
        
        dev->popData();
    }
    
    static std::pair<std::string, int> mockResetCallback(device* dev) {
        std::cout << "[MockLaddar] 执行重启..." << std::endl;
        
        LaddarServerMock* mockServer = static_cast<LaddarServerMock*>(dev);
        if (mockServer) {
            mockServer->currentSeq = 0;
            std::lock_guard<std::mutex> lock(mockServer->laddarDataMutex);
            memset(&mockServer->currentLaddarData, 0, sizeof(mockServer->currentLaddarData));
        }
        
        return std::make_pair("模拟雷达服务重启成功", 0);
    }
    
    static std::pair<std::string, int> mockCheckCallback(device* dev) {
        dev->feedWatchdog();
        return std::make_pair("模拟雷达服务正常", 0);
    }
    
    // UDP接收回调
    static void udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
        // 原始数据调试
        std::cout << "[MockLaddar] UDP收到数据包: size=" << size 
                  << " from " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
        
        if (!buf || size <= 0) {
            std::cout << "[MockLaddar] 数据包为空或大小无效" << std::endl;
            return;
        }
        
        // 打印前32字节的原始数据（frame头部）
        if (size >= 24) {
            unsigned char* rawBuf = (unsigned char*)buf;
            std::cout << "[MockLaddar] Frame头部(前24字节): ";
            for (int i = 0; i < std::min(24, size); i++) {
                printf("%02X ", rawBuf[i]);
            }
            std::cout << std::endl;
        }
        
        struct frame* frameData = (struct frame*)buf;
        std::cout << "[MockLaddar] Frame解析: source=" << frameData->source 
                  << " dest=" << frameData->dest 
                  << " type=" << frameData->type 
                  << " len_data=" << frameData->len_data << std::endl;
        device* dev = (device*)net->getPrivateData();
        LaddarServerMock* mockServer = static_cast<LaddarServerMock*>(dev);
        
        if (!mockServer) return;
        
        // 处理心跳帧
        if (frameData->type == FRAME_TYPE_HEARTBEAT) {
            struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
            if (hbFrame->type == HEARTBEAT_COMMAND_RESTART && dev) {
                dev->pushData((char*)buf, size);
            }
            return;
        }
        
        // 处理数据请求帧
        if (frameData->type == FRAME_TYPE_DATA && frameData->source == ROBOT_OBJ) {
            if (frameData->len_data >= sizeof(LADDAR::requestFrameStruct)) {
                LADDAR::requestFrameStruct* request = (LADDAR::requestFrameStruct*)frameData->data;
                
                std::cout << "[MockLaddar] 收到ROBOT请求，seq: " << request->seq 
                          << " from " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
                
                // 发送模拟数据
                mockServer->handleDataRequest(*request);
            }
            
            if (dev) dev->pushData((char*)buf, size);
        }
    }
    
    void handleDataRequest(const LADDAR::requestFrameStruct& request) {
        LADDAR::laddar_data currentData;
        {
            std::lock_guard<std::mutex> lock(laddarDataMutex);
            currentData = currentLaddarData;
        }
        
        // 构造响应帧（匹配客户端期望）
        LADDAR::receiveFrameStruct response;
        response.frameType = 1;   // 匹配客户端的 frameType 字段
        response.seq = request.seq;
        response.data = currentData;
        
        // 直接使用 communicator 发送，frameType=4 (FRAME_TYPE_RESPONSE)
        if (this->communicator) {
            this->communicator->sendDataByMap(&response, sizeof(response), ROBOT_OBJ, 4);
        }
        
        static int sendCount = 0;
        sendCount++;
        if (sendCount % 50 == 0) {
            std::cout << "[MockLaddar] 已发送 " << sendCount << " 帧模拟数据到 ROBOT_OBJ" << std::endl;
        }
    }

public:
    LaddarServerMock(const char *myIp_, unsigned short myPort_,
                     const char *robotIp_, unsigned short robotPort_)
        : device("MockLaddarServer", nullptr, DeviceCallbackTable(
            LaddarServerMock::mockDataCallback,
            LaddarServerMock::mockResetCallback,
            LaddarServerMock::mockCheckCallback
        ), HTTPSERVER_OBJ)
    {
        this->myIp = std::string(myIp_);
        this->myPort = myPort_;
        this->robotIp = std::string(robotIp_);
        this->robotPort = robotPort_;
        this->currentSeq = 0;
        this->communicator = nullptr;
        this->mockDataRunning = false;
        
        memset(&this->currentLaddarData, 0, sizeof(this->currentLaddarData));
        
        std::cout << "[MockLaddar] 模拟雷达服务端已创建" << std::endl;
    }
    
    ~LaddarServerMock() {
        endWork();
        watingDeviceEnding();
    }
    
    bool startWork() {
        // 创建 UDP 通信器
        struct ipPort localAddr = {this->myIp, this->myPort};
        this->communicator = new udpTool(localAddr, MODULE_TYPE::LADDAR_OBJ);

        if (this->communicator->createNet(FRAME_MAX_SIZE) != SUCCESS) {
            std::cerr << "[MockLaddar] sock error" << std::endl;
            delete this->communicator;
            this->communicator = nullptr;
            return false;
        }
        
        // 注册ROBOT_OBJ地址映射
        this->communicator->registerDestIpMap(this->robotIp, this->robotPort, ROBOT_OBJ);
        this->communicator->setPrivateData(this);
        
        // 启动接收线程
        if (this->communicator->startRecvThread(LaddarServerMock::udpRecvCallback) != SUCCESS) {
            std::cerr << "[MockLaddar] thread start error" << std::endl;
            delete this->communicator;
            this->communicator = nullptr;
            return false;
        }
        
        // 启动心跳
        this->startHeartbeat();
        
        // 启动模拟数据生成线程
        mockDataRunning = true;
        mockDataThread = std::thread(&LaddarServerMock::generateMockData, this);
        
        std::cout << "[MockLaddar] 模拟雷达服务启动成功!" << std::endl;
        std::cout << "  - 监听端口: " << this->myPort << std::endl;
        std::cout << "  - 目标ROBOT: " << this->robotIp << ":" << this->robotPort << std::endl;
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
        
        std::cout << "[MockLaddar] 模拟雷达服务结束" << std::endl;
    }
};

// ==================== 主函数 ====================

LaddarServerMock* g_mock_laddar_server = nullptr;

void signalHandler(int signum) {
    std::cout << "\n[MockLaddar] 收到中断信号 (" << signum << ")，正在关闭..." << std::endl;
    if (g_mock_laddar_server) {
        g_mock_laddar_server->endWork();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "laddar_server_mock");
    ros::NodeHandle nh("~");
    
    std::cout << "========================================" << std::endl;
    std::cout << "  雷达服务端 - 模拟数据测试版本" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 从ROS参数服务器读取配置
    std::string laddar_ip, robot_ip;
    int laddar_port, robot_port;
    
    nh.param<std::string>("laddar_ip", laddar_ip, "192.168.1.100");
    nh.param<int>("laddar_port", laddar_port, 8080);
    nh.param<std::string>("robot_ip", robot_ip, "192.168.1.10");
    nh.param<int>("robot_port", robot_port, 9090);
    
    std::cout << "\n=== 配置信息 ===" << std::endl;
    std::cout << "雷达端地址: " << laddar_ip << ":" << laddar_port << std::endl;
    std::cout << "机器人端地址: " << robot_ip << ":" << robot_port << std::endl;
    std::cout << "数据模式: 模拟数据（圆形轨迹 + 方形房间）" << std::endl;
    
    // 创建模拟雷达服务端
    g_mock_laddar_server = new LaddarServerMock(
        laddar_ip.c_str(),
        static_cast<unsigned short>(laddar_port),
        robot_ip.c_str(),
        static_cast<unsigned short>(robot_port)
    );
    
    // 启动服务
    if (!g_mock_laddar_server->startWork()) {
        std::cerr << "[MockLaddar] 启动失败!" << std::endl;
        delete g_mock_laddar_server;
        return -1;
    }
    
    std::cout << "\n[MockLaddar] 服务已启动，等待ROBOT请求..." << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    
    // 使用ROS事件循环
    ros::Rate rate(10);  // 10Hz
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // 清理
    std::cout << "\n[MockLaddar] 正在关闭服务..." << std::endl;
    g_mock_laddar_server->endWork();
    g_mock_laddar_server->watingDeviceEnding();
    delete g_mock_laddar_server;
    g_mock_laddar_server = nullptr;
    
    std::cout << "[MockLaddar] 服务已关闭" << std::endl;
    return 0;
}
