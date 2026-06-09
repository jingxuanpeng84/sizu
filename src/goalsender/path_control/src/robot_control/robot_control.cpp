#include "path_control/include/robot_control.hpp"

// 构造函数
robotControl::robotControl(const char *serverIp_, unsigned short serverPort_, 
                      const char *myIp_, unsigned short myPort_, octiRobot* robot_)
    : device("robotControl", nullptr, DeviceCallbackTable(
        robotControl::robotDataCallback, 
        robotControl::robotResetCallback, 
        robotControl::robotCheckCallback
    ),HTTPSERVER_OBJ)
{
    this->serverIp = std::string(serverIp_);
    this->serverPort = serverPort_;
    this->myIp = std::string(myIp_);
    this->myPort = myPort_;
    this->robot = robot_;
    this->communicator = nullptr;
}

// 析构函数
robotControl::~robotControl() {
    endWork();
    watingDeviceEnding();
}

//静态回调函数实现
void robotControl::robotDataCallback(device* dev) {
    auto data = dev->frontData();
    if (data.first == nullptr || data.second <= 0) {
        std::cerr << "Received empty data in robotDataCallback" << std::endl;
        return;
    }

    struct frame* frameData = (struct frame*)data.first;
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART) {
            std::cout << "robotcontrol 收到重启命令" << std::endl;
            dev->ResetModule();
        }
    }

    dev->popData();
}

//重启回调
std::pair<std::string,int> robotControl::robotResetCallback(device* dev) {
    std::cout << "robotControl is resetting..." << std::endl;
    // 在这里执行重启逻辑，例如重新初始化设备、重置状态等
    // 这只是一个示例，实际重启逻辑可能更复杂
    return std::make_pair("RESET_SUCCESS", 0);
}

//检查回调
std::pair<std::string, int> robotControl::robotCheckCallback(device* dev) {
    // 在这里执行设备检查逻辑，例如检查连接状态、资源使用情况等
    // 这只是一个示例，实际检查逻辑可能更复杂
    return std::make_pair("CHECK_OK", 0);
}

//UDP接收回调
void robotControl::udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
    if (buf == nullptr || size <= 0) {
        std::cerr << "Received empty data in udpRecvCallback" << std::endl;
        return;
    }

    struct frame* frameData = (struct frame*)buf;
    device* dev = (device*)net->getPrivateData(); 
    
    // 处理心跳帧
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART && dev) {
            std::cout << "robotcontrol 收到重启命令" << std::endl;
            dev->ResetModule();
        }
        return;
    }
    
    // 处理控制命令
    if (frameData->type == FRAME_TYPE_CONTROL || frameData->type == FRAME_TYPE_DATA) {
        if (frameData->len_data >= sizeof(robotControlData)) {
            // 解析控制数据
            robotControlData* controlData = (robotControlData*)frameData->data;
            
            // 获取 robotControl 对象
            robotControl* robotDev = dynamic_cast<robotControl*>(dev);
            if (robotDev) {
                // 处理控制命令
                robotDev->processControlCommand(*controlData);
            }
        }
        
        // 将数据推送到设备队列
        if (dev) dev->pushData((char*)buf, size);
    }
}


// ==================== 处理控制命令 ====================
void robotControl::processControlCommand(const robotControlData& controlData) {
    if (!this->robot) return;
    
    // 根据命令类型执行相应操作
    switch (controlData.command) {
        case OCTIROBOT::MANIPULATION::FORWARD:
            this->robot->octiMove(controlData.param3, 0, 0);  // param3 作为前进速度
            std::cout << "[RobotControl] 前进命令, 速度: " << controlData.param3 << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::BACKWARD:
            this->robot->octiMove(-controlData.param3, 0, 0);  // param3 作为后退速度
            std::cout << "[RobotControl] 后退命令, 速度: " << controlData.param3 << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::TURN_LEFT:
            this->robot->octiMove(0, 0, controlData.param3);  // param3 作为左转角速度
            std::cout << "[RobotControl] 左转命令, 角速度: " << controlData.param3 << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::TURN_RIGHT:
            this->robot->octiMove(0, 0, -controlData.param3);  // param3 作为右转角速度
            std::cout << "[RobotControl] 右转命令, 角速度: " << controlData.param3 << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::LEFT_MOVE:
            this->robot->octiMove(0, controlData.param3, 0);  // param3 作为左移速度
            std::cout << "[RobotControl] 左移命令, 速度: " << controlData.param3 << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::RIGHT_MOVE:
            this->robot->octiMove(0, -controlData.param3, 0);  // param3 作为右移速度
            std::cout << "[RobotControl] 右移命令, 速度: " << controlData.param3 << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::STAND_UP:
            this->robot->octiBalanceStand();
            std::cout << "[RobotControl] 站立命令" << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::STAND_DOWN:
            this->robot->octiStandUp();
            usleep(500000);  // 等待0.5秒
            this->robot->octiStandDown();
            std::cout << "[RobotControl] 趴下命令" << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::START_INSPECTION:
            this->robot->octiBalanceStand();
            this->robot->setRobotController(OCTIROBOT::ROBOTCONTROLLER::NAVIGATION);
            std::cout << "[RobotControl] 开始巡检命令" << std::endl;
            break;
            
        case OCTIROBOT::MANIPULATION::STOP_INSPECTION:
            this->robot->setRobotController(OCTIROBOT::ROBOTCONTROLLER::VOICEINTERACTION);
            std::cout << "[RobotControl] 停止巡检命令" << std::endl;
            break;
            
        default:
            std::cout << "[RobotControl] 未知命令: " << controlData.command << std::endl;
            break;
    }
}

// ==================== 启动工作 ====================
bool robotControl::startWork()
{
    // 1. 创建 UDP 通信器
    struct ipPort localAddr = {this->myIp, this->myPort};
    this->communicator = new udpTool(localAddr, MODULE_TYPE::ROBOT_OBJ);

    // 2. 创建网络
    if (this->communicator->createNet(FRAME_MAX_SIZE) != ERROR_NUM::SUCCESS) {
        std::cerr << "[RobotControl] Failed to create network" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 3. 注册远程服务器地址
    this->communicator->registerDestIpMap(this->serverIp, this->serverPort, HTTPSERVER_OBJ);
    
    // 4. 设置私有数据
    this->communicator->setPrivateData(this);
    
    // 5. 启动接收线程
    if (this->communicator->startRecvThread(robotControl::udpRecvCallback) != ERROR_NUM::SUCCESS) {
        std::cerr << "[RobotControl] Failed to start receive thread" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }

    // 6. 启动心跳
    this->startHeartbeat();
    
    std::cout << "[RobotControl] Device started successfully" << std::endl;
    return true;
}

// ==================== 结束工作 ====================
bool robotControl::endWork()
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
void robotControl::watingDeviceEnding()
{
    // 清理通信器
    if (this->communicator) {
        this->communicator->destroryNet();
        delete this->communicator;
        this->communicator = nullptr;
    }
    
    std::cout << "[RobotControl] Device finished" << std::endl;
}