#include "path_control/include/send.hpp"
// ==================== 构造函数 ====================
sendDevice::sendDevice(const char *deviceIp_, unsigned short devicePort_, const char *myIp_, unsigned short myPort_)
    : device("SendDevice", nullptr, DeviceCallbackTable(
        sendDevice::sendDataCallback,
        sendDevice::sendResetCallback,
        sendDevice::sendCheckCallback
    ), HTTPSERVER_OBJ)
{
    // 转换为 std::string（安全存储）
    this->deviceIP = std::string(deviceIp_);
    this->devicePort = devicePort_;
    this->myIp = std::string(myIp_);
    this->myPort = myPort_;
    
    this->communicator = nullptr;
}

// ==================== 析构函数 ====================
sendDevice::~sendDevice()
{
    endWork();
    watingDeviceEnding();
}

// ==================== 静态回调函数 ====================
// 数据处理回调
void sendDevice::sendDataCallback(device* dev) {
    auto data = dev->frontData();
    if (data.first == nullptr) return;
    
    struct frame* frameData = (struct frame*)data.first;
    
    // 处理心跳帧中的重启命令
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART) {
            std::cout << "[Send] 收到重启命令" << std::endl;
            dev->ResetModule();
        }
    }
    
    dev->popData();
}

// 重启回调
std::pair<std::string, int> sendDevice::sendResetCallback(device* dev) {
    std::cout << "[Send] 执行重启..." << std::endl;
    
    sendDevice* sendDev = dynamic_cast<sendDevice*>(dev);
    if (sendDev) {
        // 这里可以添加重置逻辑，例如清空数据缓冲等
    }
    
    return std::make_pair("Reset Done", 0);
}

// 状态检查回调
std::pair<std::string, int> sendDevice::sendCheckCallback(device* dev) {
    // 这里可以添加状态检查逻辑，例如检查通信状态、数据有效性等
    return std::make_pair("Status OK", 0);
}

// ==================== UDP 接收回调 ====================
void sendDevice::udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
    if (!buf || size <= 0) return;
    
    struct frame* frameData = (struct frame*)buf;
    device* dev = (device*)net->getPrivateData();
    
    // 处理心跳帧
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART && dev) {
            dev->pushData((char*)buf, size);
        }
        return;
    }
    
// 处理其他类型的帧（如数据帧）
    if (frameData->type == FRAME_TYPE_DATA || frameData->type == FRAME_TYPE_RESPONSE) {
        // 这里可以添加对数据帧的处理逻辑，例如解析路径点数据等
    }
    
    if (dev) dev->pushData((char*)buf, size);
}

// ==================== 启动工作 ====================
bool sendDevice::startWork()
{
    // 1. 创建 UDP 通信器
    struct ipPort localAddr = 
    {
        this -> myIp,
        this -> myPort
    };
    
    this->communicator = new udpTool(
        localAddr,
        MODULE_TYPE::ROBOT_OBJ
    );
    
    // 2. 创建网络
    if (this->communicator->createNet(FRAME_MAX_SIZE) != ERROR_NUM::SUCCESS) {
        std::cerr << "[Send] Failed to create network" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 3. 注册远程地址
    this->communicator->registerDestIpMap(this->deviceIP, this->devicePort, MODULE_TYPE::HTTPSERVER_OBJ);
    // 4. 设置私有数据
    this->communicator->setPrivateData(this);

    // 5. 启动接收线程
    if (this->communicator->startRecvThread(sendDevice::udpRecvCallback) != ERROR_NUM::SUCCESS) {
        std::cerr << "[Send] Failed to start receive thread" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }

    //启动心跳
    this->startHeartbeat();
    std::cout << "[Send] Device started successfully" << std::endl;
    
    return true;
}

// ==================== 结束工作 ====================
bool sendDevice::endWork()
{
    // 停止心跳
    this->stopHeartbeat();
    
    // 停止接收线程
    if (this->communicator) {
        this->communicator->endRecvThread();
    }

    return true;
}

//等待结束
void sendDevice::watingDeviceEnding()
{
    // 清理通信器
    if (this->communicator) {
        this->communicator->destroryNet();
        delete this->communicator;
        this->communicator = nullptr;
    }
    
    std::cout << "[Send] Device finished" << std::endl;
}

// 发送路径点
bool sendDevice::sendPathPoint(uint32_t sequence, uint32_t keyPointType, float x, float y) {

    pathPointData pointData;
    pointData.sequence = sequence;
    pointData.keyPointType = keyPointType;
    pointData.x = x;
    pointData.y = y;

    this->sendDataTo(&pointData, sizeof(pointData), HTTPSERVER_OBJ);
    return true;

}

// 发送时间戳
bool sendDevice::sendTimestamp(uint32_t flag1, uint32_t flag2, float timestamp1, float timestamp2) {
    timestampData data;
    data.flag1 = flag1;
    data.flag2 = flag2;
    data.timestamp1 = timestamp1;
    data.timestamp2 = timestamp2;
    
    // 使用继承的 sendDataTo 方法
    this->sendDataTo(&data, sizeof(data), HTTPSERVER_OBJ);
    
    return true;
}

