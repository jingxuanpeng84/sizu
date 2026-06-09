#include "/home/orangepi/slam_ws/src/goalsender/include/laddar_server_newdevice.hpp"
#include <iostream>
#include <cstring>
#include <signal.h>

// ==================== 构造函数 ====================
LaddarServerNewDevice::LaddarServerNewDevice(const char *myIp_, unsigned short myPort_,
                                           const char *robotIp_, unsigned short robotPort_,
                                           ros::NodeHandle* nh)
    : device("LaddarServer", nullptr, DeviceCallbackTable(
        LaddarServerNewDevice::laddarServerDataCallback,
        LaddarServerNewDevice::laddarServerResetCallback,
        LaddarServerNewDevice::laddarServerCheckCallback
    ), HTTPSERVER_OBJ)
{
    this->myIp = std::string(myIp_);
    this->myPort = myPort_;
    this->robotIp = std::string(robotIp_);
    this->robotPort = robotPort_;
    
    this->currentSeq = 0;
    this->communicator = nullptr;
    
    // ROS相关初始化
    this->nh_ = nh;
    this->laser_received_ = false;
    this->odom_received_ = false;
    this->laser_topic_ = "/scan";
    this->odom_topic_ = "/odom";
    
    // 初始化雷达数据
    memset(&this->currentLaddarData, 0, sizeof(this->currentLaddarData));
    
    // 从参数服务器读取话题名称
    if (this->nh_) {
        this->nh_->param<std::string>("laser_topic", this->laser_topic_, "/scan");
        this->nh_->param<std::string>("odom_topic", this->odom_topic_, "/odom");
        
        // 订阅话题
        this->laser_sub_ = this->nh_->subscribe(this->laser_topic_, 10,
            &LaddarServerNewDevice::laserCallback, this);
        this->odom_sub_ = this->nh_->subscribe(this->odom_topic_, 10,
            &LaddarServerNewDevice::odomCallback, this);
        
        std::cout << "[LaddarServer] ROS数据源已启用" << std::endl;
        std::cout << "  - 激光雷达话题: " << this->laser_topic_ << std::endl;
        std::cout << "  - 里程计话题: " << this->odom_topic_ << std::endl;
    } else {
        std::cerr << "[LaddarServer] 错误: 必须提供ROS节点句柄!" << std::endl;
    }
}

// ==================== 析构函数 ====================
LaddarServerNewDevice::~LaddarServerNewDevice()
{
    endWork();
    watingDeviceEnding();
}

// ==================== 静态回调函数 ====================

// 数据处理回调
void LaddarServerNewDevice::laddarServerDataCallback(device* dev) {
    auto data = dev->frontData();
    if (data.first == nullptr) return;
    
    struct frame* frameData = (struct frame*)data.first;
    
    // 处理心跳帧中的重启命令
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART) {
            std::cout << "[LaddarServer] 收到重启命令" << std::endl;
            dev->ResetModule();
        }
    }
    
    dev->popData();
}

// 重启回调
std::pair<std::string, int> LaddarServerNewDevice::laddarServerResetCallback(device* dev) {
    std::cout << "[LaddarServer] 执行重启..." << std::endl;
    
    LaddarServerNewDevice* laddarServer = static_cast<LaddarServerNewDevice*>(dev);
    if (laddarServer) {
        laddarServer->currentSeq = 0;
        // 重置雷达数据
        std::lock_guard<std::mutex> lock(laddarServer->laddarDataMutex);
        memset(&laddarServer->currentLaddarData, 0, sizeof(laddarServer->currentLaddarData));
    }
    
    return std::make_pair("雷达服务重启成功", 0);
}

// 状态检查回调
std::pair<std::string, int> LaddarServerNewDevice::laddarServerCheckCallback(device* dev) {
    LaddarServerNewDevice* laddarServer = static_cast<LaddarServerNewDevice*>(dev);
    
    if (laddarServer && (!laddarServer->laser_received_ || !laddarServer->odom_received_)) {
        return std::make_pair("等待ROS数据", -1);
    }
    
    // 正常喂狗
    dev->feedWatchdog();
    return std::make_pair("雷达服务正常", 0);
}

// ==================== UDP 接收回调 ====================
void LaddarServerNewDevice::udpServerRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
    if (!buf || size <= 0) return;
    
    struct frame* frameData = (struct frame*)buf;
    device* dev = (device*)net->getPrivateData();
    LaddarServerNewDevice* laddarServer = static_cast<LaddarServerNewDevice*>(dev);
    
    if (!laddarServer) return;
    
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
            // 解析请求
            LADDAR::requestFrameStruct* request = (LADDAR::requestFrameStruct*)frameData->data;
            
            std::cout << "[LaddarServer] 收到ROBOT请求，seq: " << request->seq 
                      << " from " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
            
            // 处理请求
            laddarServer->handleDataRequest(*request, addr);
        }
        
        if (dev) dev->pushData((char*)buf, size);
    }
}

// ==================== 启动工作 ====================
bool LaddarServerNewDevice::startWork()
{
    // 1. 创建 UDP 通信器
    struct ipPort localAddr = {this->myIp, this->myPort};
    this->communicator = new udpTool(localAddr, MODULE_TYPE::LADDAR_OBJ);

    // 2. 创建网络
    if (this->communicator->createNet(FRAME_MAX_SIZE) != SUCCESS) {
        std::cerr << "[LaddarServer] sock error" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 3. 注册ROBOT_OBJ地址映射
    this->communicator->registerDestIpMap(this->robotIp, this->robotPort, ROBOT_OBJ);
    
    // 4. 设置私有数据和通信器
    this->communicator->setPrivateData(this);
    
    // 5. 启动接收线程
    if (this->communicator->startRecvThread(LaddarServerNewDevice::udpServerRecvCallback) != SUCCESS) {
        std::cerr << "[LaddarServer] thread start error" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 6. 启动心跳
    this->startHeartbeat();
    
    std::cout << "[LaddarServer] 雷达服务启动成功! 监听端口: " << this->myPort 
              << " 目标ROBOT: " << this->robotIp << ":" << this->robotPort << std::endl;
    
    // 等待ROS数据就绪
    if (!this->laser_received_ || !this->odom_received_) {
        std::cout << "[LaddarServer] 等待ROS数据..." << std::endl;
        if (!this->laser_received_) {
            std::cout << "  - 等待激光雷达数据: " << this->laser_topic_ << std::endl;
        }
        if (!this->odom_received_) {
            std::cout << "  - 等待里程计数据: " << this->odom_topic_ << std::endl;
        }
    }
    
    return true;
}

// ==================== 结束工作 ====================
bool LaddarServerNewDevice::endWork()
{
    // 停止心跳
    this->stopHeartbeat();
    
    // 停止接收线程
    if (this->communicator) {
        this->communicator->endRecvThread();
    }
    
    return true;
}

// ==================== 等待结束 ====================
void LaddarServerNewDevice::watingDeviceEnding()
{
    // 清理通信器
    if (this->communicator) {
        this->communicator->destroryNet();
        delete this->communicator;
        this->communicator = nullptr;
    }
    
    std::cout << "[LaddarServer] 雷达服务结束" << std::endl;
}

// ==================== 数据接口 ====================
void LaddarServerNewDevice::updateLaddarData(const LADDAR::laddar_data& data) {
    std::lock_guard<std::mutex> lock(this->laddarDataMutex);
    this->currentLaddarData = data;
}

LADDAR::laddar_data LaddarServerNewDevice::getCurrentLaddarData() {
    std::lock_guard<std::mutex> lock(this->laddarDataMutex);
    return this->currentLaddarData;
}

// ==================== 请求处理 ====================
void LaddarServerNewDevice::handleDataRequest(const LADDAR::requestFrameStruct& request, sockaddr_in clientAddr) {
    // 获取当前雷达数据
    LADDAR::laddar_data currentData = getCurrentLaddarData();
    
    // 发送雷达数据
    sendLaddarData(currentData, request.seq, clientAddr);
}

void LaddarServerNewDevice::sendLaddarData(const LADDAR::laddar_data& data, unsigned long seq, sockaddr_in clientAddr) {
    // 构造响应帧（匹配客户端期望的结构）
    LADDAR::receiveFrameStruct response;
    response.frameType = 1;   // 匹配客户端的 frameType 字段
    response.seq = seq;
    response.data = data;
    
    // 直接使用 communicator 发送，frameType=4 (FRAME_TYPE_RESPONSE)
    if (this->communicator) {
        this->communicator->sendDataByMap(&response, sizeof(response), ROBOT_OBJ, 4);
    }
    
    static int sendCount = 0;
    sendCount++;
    if (sendCount % 50 == 0) {
        std::cout << "[LaddarServer] 已发送 " << sendCount << " 帧数据到 ROBOT_OBJ" << std::endl;
    }
}

// ==================== ROS回调函数 ====================

/**
 * @brief 激光雷达数据回调
 */
void LaddarServerNewDevice::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->laddarDataMutex);
    
    // 转换激光数据到固定大小数组
    int scan_size = msg->ranges.size();
    
    if (scan_size <= LINE_NUM) {
        // 如果扫描点数少于360，直接复制并填充
        for (int i = 0; i < scan_size; i++) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
                this->currentLaddarData.dist[i] = range;
            } else {
                this->currentLaddarData.dist[i] = msg->range_max;
            }
        }
        // 填充剩余部分
        for (int i = scan_size; i < LINE_NUM; i++) {
            this->currentLaddarData.dist[i] = msg->range_max;
        }
    } else {
        // 如果扫描点数多于360，进行降采样
        double step = (double)scan_size / LINE_NUM;
        for (int i = 0; i < LINE_NUM; i++) {
            int index = (int)(i * step);
            float range = msg->ranges[index];
            if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
                this->currentLaddarData.dist[i] = range;
            } else {
                this->currentLaddarData.dist[i] = msg->range_max;
            }
        }
    }
    
    this->laser_received_ = true;
    
    // 定期打印接收状态
    static int count = 0;
    if (++count % 100 == 0) {
        std::cout << "[LaddarServer] 激光数据更新 - 点数: " << scan_size 
                  << ", 范围: [" << msg->range_min << ", " << msg->range_max << "]m" << std::endl;
    }
}

/**
 * @brief 里程计数据回调
 */
void LaddarServerNewDevice::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->laddarDataMutex);
    
    // 更新位置信息
    this->currentLaddarData.x = msg->pose.pose.position.x;
    this->currentLaddarData.y = msg->pose.pose.position.y;
    
    // 计算yaw角
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    this->currentLaddarData.yaw = yaw;
    
    this->odom_received_ = true;
    
    // 定期打印位置信息
    static int count = 0;
    if (++count % 100 == 0) {
        std::cout << "[LaddarServer] 位置更新 - 坐标: (" << this->currentLaddarData.x 
                  << ", " << this->currentLaddarData.y << "), 航向: " << this->currentLaddarData.yaw 
                  << " rad (" << (this->currentLaddarData.yaw * 180.0 / M_PI) << " deg)" << std::endl;
    }
}

// ==================== ROS配置函数 ====================

/**
 * @brief 设置ROS话题名称
 */
void LaddarServerNewDevice::setROSTopics(const std::string& laser_topic, const std::string& odom_topic) {
    if (!this->nh_) {
        std::cerr << "[LaddarServer] 错误: 未提供ROS节点句柄" << std::endl;
        return;
    }
    
    this->laser_topic_ = laser_topic;
    this->odom_topic_ = odom_topic;
    
    // 重新订阅话题
    this->laser_sub_.shutdown();
    this->odom_sub_.shutdown();
    
    this->laser_sub_ = this->nh_->subscribe(this->laser_topic_, 10,
        &LaddarServerNewDevice::laserCallback, this);
    this->odom_sub_ = this->nh_->subscribe(this->odom_topic_, 10,
        &LaddarServerNewDevice::odomCallback, this);
    
    std::cout << "[LaddarServer] ROS话题已更新" << std::endl;
    std::cout << "  - 激光雷达话题: " << this->laser_topic_ << std::endl;
    std::cout << "  - 里程计话题: " << this->odom_topic_ << std::endl;
}

/**
 * @brief 检查ROS数据是否就绪
 */
bool LaddarServerNewDevice::isROSDataReady() const {
    return this->laser_received_ && this->odom_received_;
}

// ==================== ROS节点主函数 ====================

// 全局指针，用于信号处理
LaddarServerNewDevice* g_laddar_server = nullptr;

/**
 * @brief 信号处理函数
 */
void signalHandler(int signum) {
    ROS_INFO("收到中断信号 (%d)，正在关闭雷达服务...", signum);
    if (g_laddar_server) {
        g_laddar_server->endWork();
    }
    ros::shutdown();
}

/**
 * @brief ROS节点主函数
 */
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "laddar_server_node");
    ros::NodeHandle nh("~");
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 读取参数
    std::string laddar_ip, robot_ip;
    int laddar_port, robot_port;
    std::string laser_topic, odom_topic;
    
    nh.param<std::string>("laddar_ip", laddar_ip, "192.168.1.100");
    nh.param<int>("laddar_port", laddar_port, 8080);
    nh.param<std::string>("robot_ip", robot_ip, "192.168.1.10");
    nh.param<int>("robot_port", robot_port, 9090);
    nh.param<std::string>("laser_topic", laser_topic, "/scan");
    nh.param<std::string>("odom_topic", odom_topic, "/odom");
    
    ROS_INFO("=== 雷达服务端配置 ===");
    ROS_INFO("雷达端地址: %s:%d", laddar_ip.c_str(), laddar_port);
    ROS_INFO("机器人端地址: %s:%d", robot_ip.c_str(), robot_port);
    ROS_INFO("激光雷达话题: %s", laser_topic.c_str());
    ROS_INFO("里程计话题: %s", odom_topic.c_str());
    
    // 创建雷达服务端实例
    g_laddar_server = new LaddarServerNewDevice(
        laddar_ip.c_str(), 
        static_cast<unsigned short>(laddar_port),
        robot_ip.c_str(), 
        static_cast<unsigned short>(robot_port),
        &nh
    );
    
    // 设置ROS话题（如果与默认不同）
    if (laser_topic != "/scan" || odom_topic != "/odom") {
        g_laddar_server->setROSTopics(laser_topic, odom_topic);
    }
    
    // 启动雷达服务
    if (!g_laddar_server->startWork()) {
        ROS_ERROR("雷达服务启动失败!");
        delete g_laddar_server;
        return -1;
    }
    
    ROS_INFO("雷达服务已启动，等待ROS数据和ROBOT请求...");
    
    // 进入ROS事件循环
    ros::spin();
    
    // 清理
    ROS_INFO("正在关闭雷达服务...");
    g_laddar_server->endWork();
    g_laddar_server->watingDeviceEnding();
    delete g_laddar_server;
    g_laddar_server = nullptr;
    
    ROS_INFO("雷达服务已关闭");
    return 0;
}
