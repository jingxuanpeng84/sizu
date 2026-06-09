#include "path_control/include/laddar_device.hpp"

// ==================== 构造函数 ====================
laddarDevice::laddarDevice(const char *deviceIp_, unsigned short devicePort_, 
                           const char *myIp_, unsigned short myPort_)
    : device("LaddarDevice", nullptr, DeviceCallbackTable(
        laddarDevice::laddarDataCallback,
        laddarDevice::laddarResetCallback,
        laddarDevice::laddarCheckCallback
    ), HTTPSERVER_OBJ)
{
    // 转换为 std::string（安全存储）
    this->deviceIP = std::string(deviceIp_);
    this->devicePort = devicePort_;
    this->myIp = std::string(myIp_);
    this->myPort = myPort_;
    
    this->DataUpdateFlag = false;
    this->laddarBeginFlag = false;
    this->requestThreadRunning = false;
    this->communicator = nullptr;
    this->requestThread = nullptr;
}

// ==================== 析构函数 ====================
laddarDevice::~laddarDevice()
{
    endWork();
    watingDeviceEnding();
}

// ==================== 静态回调函数 ====================

// 数据处理回调
void laddarDevice::laddarDataCallback(device* dev) {
    auto data = dev->frontData();
    if (data.first == nullptr) return;
    
    struct frame* frameData = (struct frame*)data.first;
    
    // 处理心跳帧中的重启命令
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART) {
            std::cout << "[Laddar] 收到重启命令" << std::endl;
            dev->ResetModule();
        }
    }
    
    dev->popData();
}

// 重启回调
std::pair<std::string, int> laddarDevice::laddarResetCallback(device* dev) {
    std::cout << "[Laddar] 执行重启..." << std::endl;
    
    laddarDevice* laddarDev = dynamic_cast<laddarDevice*>(dev);
    if (laddarDev) {
        laddarDev->DataUpdateFlag = false;
        laddarDev->laddarBeginFlag = false;
    }
    
    return std::make_pair("雷达重启成功", 0);
}

// 状态检查回调
std::pair<std::string, int> laddarDevice::laddarCheckCallback(device* dev) {
    laddarDevice* laddarDev = dynamic_cast<laddarDevice*>(dev);
    
    if (laddarDev && !laddarDev->DataUpdateFlag) {
        return std::make_pair("雷达数据未更新", -1);
    }
    
    // 正常喂狗
    dev->feedWatchdog();
    return std::make_pair("雷达正常", 0);
}

// ==================== UDP 接收回调 ====================
void laddarDevice::udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
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
    
    // 处理雷达数据帧
    if (frameData->type == FRAME_TYPE_DATA || frameData->type == FRAME_TYPE_RESPONSE) {
        if (frameData->len_data >= sizeof(LADDAR::receiveFrameStruct)) {
            // 解析雷达数据
            LADDAR::receiveFrameStruct* laddarFrame = (LADDAR::receiveFrameStruct*)frameData->data;
            
            // 获取 laddarDevice 对象
            laddarDevice* laddarDev = dynamic_cast<laddarDevice*>(dev);
            if (laddarDev) {
                // 更新数据
                std::lock_guard<std::mutex> lock(laddarDev->DataAccessMutex);
                laddarDev->dataBuf = laddarFrame->data;
                laddarDev->DataUpdateFlag = true;
                laddarDev->laddarBeginFlag = true;
                
                // 打印调试信息
                static int frameCount = 0;
                frameCount++;
                if (frameCount == 1) {
                    std::cout << "[Laddar] lidar begin" << std::endl;
                }
                if (frameCount % 100 == 0) {
                    std::cout << "[Laddar] Frame " << frameCount 
                              << " | Pos: (" << laddarFrame->data.x 
                              << ", " << laddarFrame->data.y 
                              << ") | Yaw: " << laddarFrame->data.yaw << std::endl;
                }
            }
        }
        
        if (dev) dev->pushData((char*)buf, size);
    }
}

// ==================== 请求线程====================
void laddarDevice::requestThreadFunc() {
    std::cout << "[Laddar] device thread begin!" << std::endl;
    
    // Wait for network initialization to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Check if communicator is initialized
    if (!communicator) {
        std::cerr << "[Laddar] ERROR: communicator not initialized in request thread!" << std::endl;
        return;
    }
    
    std::cout << "[Laddar] Request thread starting with valid communicator" << std::endl;
    
    unsigned long seq = 0;
    LADDAR::requestFrameStruct send_data;
    
    while (requestThreadRunning) {
        // Check communicator validity before each send
        if (!communicator) {
            std::cerr << "[Laddar] ERROR: communicator became null during operation!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // 发送请求帧
        send_data.frameType = 1;
        send_data.seq = seq;
        
        // 使用 sendDataByMap 自动包装 frame 头部（source=ROBOT_OBJ, dest=LADDAR_OBJ, type=FRAME_TYPE_DATA）
        int result = communicator->sendDataByMap(&send_data, sizeof(send_data), LADDAR_OBJ, FRAME_TYPE_DATA);
        
        // 定期打印发送状态（包括失败情况）
        static int sendCount = 0;
        sendCount++;
        if (sendCount <= 5 || sendCount % 100 == 0) {
            std::cout << "[Laddar] 发送请求 #" << sendCount << ", seq=" << seq 
                      << ", result=" << result 
                      << " to " << this->deviceIP << ":" << this->devicePort << std::endl;
        }
        
        if (result < 0) {
            std::cerr << "[Laddar] 发送失败, result=" << result << std::endl;
        }
        
        // 如果已经开始接收数据，递增序列号
        if (this->laddarBeginFlag) {
            seq = (seq + 1) % LADDAR_MAX_SEQ;
        }
        
        // 50ms 发送一次请求
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }
    
    std::cout << "[Laddar] device thread exit" << std::endl;
}

// ==================== 启动工作 ====================
bool laddarDevice::startWork()
{
    // 1. 创建 UDP 通信器
    struct ipPort localAddr = {this->myIp, this->myPort};
    this->communicator = new udpTool(localAddr, MODULE_TYPE::ROBOT_OBJ);

    // 2. 创建网络
    if (this->communicator->createNet(FRAME_MAX_SIZE) != SUCCESS) {
        std::cerr << "[Laddar] sock error" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 3. 注册远程雷达地址
    this->communicator->registerDestIpMap(this->deviceIP, this->devicePort, LADDAR_OBJ);
    
    // 4. 设置私有数据
    this->communicator->setPrivateData(this);
    
    // 5. 启动接收线程
    if (this->communicator->startRecvThread(laddarDevice::udpRecvCallback) != SUCCESS) {
        std::cerr << "[Laddar] thread start error" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 6. 启动心跳
    this->startHeartbeat();
    
    // 7. 启动请求线程（替代原来的 deviceWorkTHreadFunc）
    this->requestThreadRunning = true;
    this->requestThread = new std::thread(&laddarDevice::requestThreadFunc, this);
    
    std::cout << "[Laddar] Device start success!" << std::endl;
    return true;
}

// ==================== 结束工作 ====================
bool laddarDevice::endWork()
{
    this->requestThreadRunning = false;
    
    // 停止请求线程
    if (this->requestThread && this->requestThread->joinable()) {
        this->requestThread->join();
        delete this->requestThread;
        this->requestThread = nullptr;
    }
    
    // 停止心跳
    this->stopHeartbeat();
    
    // 停止接收线程
    if (this->communicator) {
        this->communicator->endRecvThread();
    }
    
    return true;
}

// ==================== 等待结束 ====================
void laddarDevice::watingDeviceEnding()
{
    // 清理通信器
    if (this->communicator) {
        this->communicator->destroryNet();
        delete this->communicator;
        this->communicator = nullptr;
    }
    
    std::cout << "[Laddar] Device finished" << std::endl;
}

// ==================== 读取数据 ====================
bool laddarDevice::readDeviceData(void *dataBuf, unsigned int &dataSize)
{
    unsigned int temptTimes = 0;
    while (true)
    {
        if (this->DataUpdateFlag == true)
        {
            std::lock_guard<std::mutex> lock(this->DataAccessMutex);
            memcpy(dataBuf, &(this->dataBuf), sizeof(this->dataBuf));
            this->DataUpdateFlag = false;
            return true;
        }
        else
        {
            temptTimes++;
            if(temptTimes >= 20)
            {
                std::cout << "[Laddar] device may disconnect!" << std::endl;
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }
    
    return true;
}

// ==================== 发送数据 ====================
bool laddarDevice::sendDeviceData(void *dataBuf, unsigned int dataSize)
{
    // 使用继承的 sendDataTo 方法
    this->sendDataTo(dataBuf, dataSize, LADDAR_OBJ);
    return true;
}

// ==================== 检查是否开始工作 ====================
bool laddarDevice::isDeviceBeginWorking()
{
    return this->laddarBeginFlag;
}
