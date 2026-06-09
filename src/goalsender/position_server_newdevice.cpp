#include "position_server_newdevice.hpp"
#include <iostream>
#include <cstring>
#include <signal.h>

// 全局指针，用于信号处理
PositionServerNewDevice* g_position_server = nullptr;

// ==================== 构造函数 ====================
PositionServerNewDevice::PositionServerNewDevice(const char *myIp_, unsigned short myPort_,
                                               const char *httpServerIp_, unsigned short httpServerPort_,
                                               ros::NodeHandle* nh)
    : device("PositionServer", nullptr, DeviceCallbackTable(
        PositionServerNewDevice::positionServerDataCallback,
        PositionServerNewDevice::positionServerResetCallback,
        PositionServerNewDevice::positionServerCheckCallback
    ), LADDAR_OBJ)  // 作为LADDAR_OBJ模块运行
{
    this->myIp = std::string(myIp_);
    this->myPort = myPort_;
    this->httpServerIp = std::string(httpServerIp_);
    this->httpServerPort = httpServerPort_;
    
    this->currentSeq = 0;
    this->communicator = nullptr;
    
    // ROS相关初始化
    this->nh_ = nh;
    this->odom_received_ = false;
    this->odom_topic_ = "/Odometry";
    this->source_frame_ = "/robot_body";
    this->target_frame_ = "/map";
    this->use_tf_ = true;  // 默认使用TF而不是Odometry
    
    // 初始化位置数据
    memset(&this->currentPositionData, 0, sizeof(this->currentPositionData));

    // 初始化地图参数为默认值
    this->mapParams.resolution = 0.02;
    this->mapParams.origin_x = -5.96;
    this->mapParams.origin_y = -6.02;
    this->mapParams.width = 835;
    this->mapParams.height = 464;
    this->current_map_id_ = 1;  // 默认地图1
    
    // 尝试从FTP目录加载最新地图参数
    std::string ftp_yaml = findLatestMapYAML();
    if (!ftp_yaml.empty() && loadMapParamsFromYAML(ftp_yaml)) {
        std::cout << "[PositionServer] 已从FTP目录加载地图参数: " << ftp_yaml << std::endl;
    } else {
        std::cout << "[PositionServer] 使用默认地图参数" << std::endl;
    }
    
    // 加载地图编号
    this->current_map_id_ = loadCurrentMapId();
    
    // 🔴 立即初始化 currentPositionData.map_id
    this->currentPositionData.map_id = this->current_map_id_;
    
    std::cout << "[PositionServer] 地图参数已加载:" << std::endl;
    std::cout << "  - 地图编号: " << this->current_map_id_ << std::endl;
    std::cout << "  - 分辨率: " << this->mapParams.resolution << " m/pixel" << std::endl;
    std::cout << "  - 原点: (" << this->mapParams.origin_x << ", " << this->mapParams.origin_y << ")" << std::endl;
    std::cout << "  - 尺寸: " << this->mapParams.width << " x " << this->mapParams.height << " pixels" << std::endl;
    
    // 从参数服务器读取话题名称
    if (this->nh_) {
        this->nh_->param<std::string>("odom_topic", this->odom_topic_, "/Odometry");
        this->nh_->param<std::string>("source_frame", this->source_frame_, "/robot_body");
        this->nh_->param<std::string>("target_frame", this->target_frame_, "/map");
        this->nh_->param<bool>("use_tf", this->use_tf_, true);
        
        if (this->use_tf_) {
            // 使用TF模式
            std::cout << "[PositionServer] 使用TF模式获取位置" << std::endl;
            std::cout << "  - 源坐标系: " << this->source_frame_ << std::endl;
            std::cout << "  - 目标坐标系: " << this->target_frame_ << std::endl;
            
            // 启动TF更新线程
            startTFUpdateThread();
        } else {
            // 使用Odometry模式
            std::cout << "[PositionServer] 使用Odometry模式获取位置" << std::endl;
            std::cout << "  - 里程计话题: " << this->odom_topic_ << std::endl;
            
            // 订阅话题
            this->odom_sub_ = this->nh_->subscribe(this->odom_topic_, 10,
                &PositionServerNewDevice::odomCallback, this);
        }
        
        // 启动文件监听线程
        startMapFileWatcher();
    } else {
        std::cerr << "[PositionServer] 错误: 必须提供ROS节点句柄!" << std::endl;
    }
}

// ==================== 析构函数 ====================
PositionServerNewDevice::~PositionServerNewDevice()
{
    stopTFUpdateThread();      // 停止TF更新线程
    stopMapFileWatcher();      // 停止文件监听线程
    endWork();
    watingDeviceEnding();
}

// ==================== 静态回调函数 ====================

// 数据处理回调
void PositionServerNewDevice::positionServerDataCallback(device* dev) {
    auto data = dev->frontData();
    if (data.first == nullptr) return;
    
    struct frame* frameData = (struct frame*)data.first;
    
    // 处理心跳帧中的重启命令
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART) {
            std::cout << "[PositionServer] 收到重启命令" << std::endl;
            dev->ResetModule();
        }
    }
    
    dev->popData();
}

// 重启回调
std::pair<std::string, int> PositionServerNewDevice::positionServerResetCallback(device* dev) {
    std::cout << "[PositionServer] 执行重启..." << std::endl;
    
    PositionServerNewDevice* positionServer = static_cast<PositionServerNewDevice*>(dev);
    if (positionServer) {
        positionServer->currentSeq = 0;
        // 重置位置数据
        std::lock_guard<std::mutex> lock(positionServer->positionDataMutex);
        memset(&positionServer->currentPositionData, 0, sizeof(positionServer->currentPositionData));
    }
    
    return std::make_pair("位置服务重启成功", 0);
}

// 状态检查回调
std::pair<std::string, int> PositionServerNewDevice::positionServerCheckCallback(device* dev) {
    PositionServerNewDevice* positionServer = static_cast<PositionServerNewDevice*>(dev);
    
    if (positionServer && !positionServer->odom_received_) {
        return std::make_pair("等待ROS位置数据", -1);
    }
    
    // 正常喂狗
    dev->feedWatchdog();
    return std::make_pair("位置服务正常", 0);
}

// ==================== UDP 接收回调 ====================
void PositionServerNewDevice::udpServerRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
    // 原始数据调试
    std::cout << "[PositionServer] UDP收到数据包: size=" << size 
              << " from " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
    
    if (!buf || size <= 0) {
        std::cout << "[PositionServer] 数据包为空或大小无效" << std::endl;
        return;
    }
    
    // 打印前24字节的原始数据（frame头部）
    if (size >= 24) {
        unsigned char* rawBuf = (unsigned char*)buf;
        std::cout << "[PositionServer] Frame头部(前24字节): ";
        for (int i = 0; i < std::min(24, size); i++) {
            printf("%02X ", rawBuf[i]);
        }
        std::cout << std::endl;
    }
    
    struct frame* frameData = (struct frame*)buf;
    std::cout << "[PositionServer] Frame解析: source=" << frameData->source 
              << " dest=" << frameData->dest 
              << " type=" << frameData->type 
              << " len_data=" << frameData->len_data << std::endl;
    
    device* dev = (device*)net->getPrivateData();
    PositionServerNewDevice* positionServer = static_cast<PositionServerNewDevice*>(dev);
    
    if (!positionServer) return;
    
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
        std::cout << "[PositionServer] 收到 INQUIRY 请求 from " 
                  << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
        
        // 直接响应到请求来源地址
        positionServer->handleInquiryRequest(addr);
        return;
    }
    
    // 处理数据请求帧（来自HTTPSERVER_OBJ）
    if (frameData->type == FRAME_TYPE_DATA && frameData->source == HTTPSERVER_OBJ) {
        if (frameData->len_data >= sizeof(POSITION::requestFrameStruct)) {
            // 解析请求
            POSITION::requestFrameStruct* request = (POSITION::requestFrameStruct*)frameData->data;
            
            std::cout << "[PositionServer] 收到HTTPSERVER请求，seq: " << request->seq 
                      << " from " << inet_ntoa(addr.sin_addr) << ":" << ntohs(addr.sin_port) << std::endl;
            
            // 处理请求
            positionServer->handleDataRequest(*request, addr);
        }
        
        if (dev) dev->pushData((char*)buf, size);
    }
}

// ==================== 启动工作 ====================
bool PositionServerNewDevice::startWork()
{
    // 1. 创建 UDP 通信器
    struct ipPort localAddr = {this->myIp, this->myPort};
    this->communicator = new udpTool(localAddr, MODULE_TYPE::LADDAR_OBJ);

    // 2. 创建网络
    if (this->communicator->createNet(FRAME_MAX_SIZE) != SUCCESS) {
        std::cerr << "[PositionServer] sock error" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 3. 注册HTTPSERVER_OBJ地址映射
    this->communicator->registerDestIpMap(this->httpServerIp, this->httpServerPort, HTTPSERVER_OBJ);
    
    // 4. 设置私有数据和通信器
    this->communicator->setPrivateData(this);
    
    // 5. 启动接收线程
    if (this->communicator->startRecvThread(PositionServerNewDevice::udpServerRecvCallback) != SUCCESS) {
        std::cerr << "[PositionServer] thread start error" << std::endl;
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 6. 启动心跳
    this->startHeartbeat();
    
    std::cout << "[PositionServer] 位置服务启动成功! 监听端口: " << this->myPort 
              << " 目标HTTPSERVER: " << this->httpServerIp << ":" << this->httpServerPort << std::endl;
    
    // 等待ROS数据就绪
    if (!this->odom_received_) {
        std::cout << "[PositionServer] 等待ROS位置数据..." << std::endl;
        std::cout << "  - 等待里程计数据: " << this->odom_topic_ << std::endl;
    }
    
    return true;
}

// ==================== 结束工作 ====================
bool PositionServerNewDevice::endWork()
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
void PositionServerNewDevice::watingDeviceEnding()
{
    // 清理通信器
    if (this->communicator) {
        this->communicator->destroryNet();
        delete this->communicator;
        this->communicator = nullptr;
    }
    
    std::cout << "[PositionServer] 位置服务结束" << std::endl;
}

// ==================== 数据接口 ====================
void PositionServerNewDevice::updatePositionData(const POSITION::position_data& data) {
    std::lock_guard<std::mutex> lock(this->positionDataMutex);
    this->currentPositionData = data;
}

POSITION::position_data PositionServerNewDevice::getCurrentPositionData() {
    std::lock_guard<std::mutex> lock(this->positionDataMutex);
    return this->currentPositionData;
}

// ==================== 请求处理 ====================

// 处理 INQUIRY 请求（响应到请求来源）
void PositionServerNewDevice::handleInquiryRequest(const sockaddr_in& clientAddr) {
    POSITION::position_data currentData = getCurrentPositionData();
    
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
        
        std::cout << "[PositionServer] 已响应 INQUIRY 到 " 
                  << clientIp.ip << ":" << clientIp.port 
                  << " 世界坐标: (" << currentData.x << ", " << currentData.y << ")"
                  << " 像素坐标: (" << currentData.px << ", " << currentData.py << ")"
                  << " 地图编号: " << currentData.map_id << std::endl;
    }
    
    delete[] sendBuf;
}

void PositionServerNewDevice::handleDataRequest(const POSITION::requestFrameStruct& request, sockaddr_in clientAddr) {
    // 获取当前位置数据
    POSITION::position_data currentData = getCurrentPositionData();
    
    // 发送位置数据
    sendPositionData(currentData, request.seq, clientAddr);
}

void PositionServerNewDevice::sendPositionData(const POSITION::position_data& data, unsigned long seq, sockaddr_in clientAddr) {
    // 构造响应帧（匹配客户端期望的结构）
    POSITION::receiveFrameStruct response;
    response.frame_type = 1;  // 匹配客户端的 frame_type 字段
    response.seq = seq;       // 使用请求的序列号
    response.data = data;     // 位置数据
    
    // 直接使用 communicator 发送，frameType=4 (FRAME_TYPE_RESPONSE)
    if (this->communicator) {
        this->communicator->sendDataByMap(&response, sizeof(response), HTTPSERVER_OBJ, 4);
    }
    
    static int sendCount = 0;
    sendCount++;
    if (sendCount % 50 == 0) {
        std::cout << "[PositionServer] 已发送 " << sendCount << " 帧位置数据到 HTTPSERVER_OBJ" << std::endl;
    }
}

// ==================== 辅助函数 ====================

/**
 * @brief 将世界坐标转换为像素坐标
 * @param world_x 世界坐标X（米）
 * @param world_y 世界坐标Y（米）
 * @param pixel_x 输出像素坐标X
 * @param pixel_y 输出像素坐标Y
 */
void PositionServerNewDevice::worldToPixel(double world_x, double world_y, int& pixel_x, int& pixel_y) {
    // 计算相对于原点的偏移（米）
    double offset_x = world_x - mapParams.origin_x;
    double offset_y = world_y - mapParams.origin_y;
    
    // 转换为像素坐标
    pixel_x = static_cast<int>(offset_x / mapParams.resolution);
    pixel_y = mapParams.height - static_cast<int>(offset_y / mapParams.resolution) - 1;
    
    // 边界检查
    if (pixel_x < 0) pixel_x = 0;
    if (pixel_x >= mapParams.width) pixel_x = mapParams.width - 1;
    if (pixel_y < 0) pixel_y = 0;
    if (pixel_y >= mapParams.height) pixel_y = mapParams.height - 1;
}

// ==================== ROS回调函数 ====================

/**
 * @brief 里程计数据回调
 */
void PositionServerNewDevice::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(this->positionDataMutex);
    
    // 更新世界坐标
    this->currentPositionData.x = msg->pose.pose.position.x;
    this->currentPositionData.y = msg->pose.pose.position.y;
    
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
    this->currentPositionData.yaw = yaw;
    
    // 计算像素坐标
    worldToPixel(this->currentPositionData.x, this->currentPositionData.y,
                 this->currentPositionData.px, this->currentPositionData.py);
    
    // 填充地图编号
    this->currentPositionData.map_id = this->current_map_id_;
    
    this->odom_received_ = true;
    
    // 定期打印位置信息（包括像素坐标和地图编号）
    static int count = 0;
    if (++count % 100 == 0) {
        std::cout << "[PositionServer] 位置更新 - 地图: " << this->currentPositionData.map_id
                  << ", 世界坐标: (" << this->currentPositionData.x 
                  << ", " << this->currentPositionData.y << "), 航向: " << this->currentPositionData.yaw 
                  << " rad (" << (this->currentPositionData.yaw * 180.0 / M_PI) << " deg)"
                  << ", 像素坐标: (" << this->currentPositionData.px << ", " << this->currentPositionData.py << ")" << std::endl;
    }
}

// ==================== ROS配置函数 ====================

/**
 * @brief 设置ROS里程计话题名称
 */
void PositionServerNewDevice::setOdomTopic(const std::string& odom_topic) {
    if (!this->nh_) {
        std::cerr << "[PositionServer] 错误: 未提供ROS节点句柄" << std::endl;
        return;
    }
    
    this->odom_topic_ = odom_topic;
    
    // 重新订阅话题
    this->odom_sub_.shutdown();
    
    this->odom_sub_ = this->nh_->subscribe(this->odom_topic_, 10,
        &PositionServerNewDevice::odomCallback, this);
    
    std::cout << "[PositionServer] ROS话题已更新" << std::endl;
    std::cout << "  - 里程计话题: " << this->odom_topic_ << std::endl;
}

/**
 * @brief 检查位置数据是否就绪
 */
bool PositionServerNewDevice::isPositionDataReady() const {
    return this->odom_received_;
}

// ==================== 信号处理和主函数 ====================

/**
 * @brief 加载当前地图编号
 */
int PositionServerNewDevice::loadCurrentMapId() {
    const std::string map_id_file = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD/.current_map_id";
    std::ifstream f(map_id_file);
    if (f.is_open()) {
        int map_id;
        f >> map_id;
        f.close();
        std::cout << "[PositionServer] 从文件读取地图编号: " << map_id << std::endl;
        return map_id;
    }
    std::cout << "[PositionServer] 地图编号文件不存在，使用默认值: 1" << std::endl;
    return 1;  // 默认地图1
}

/**
 * @brief 查找SLAM地图目录中最新的地图YAML文件
 */
std::string PositionServerNewDevice::findLatestMapYAML() {
    const std::string map_dir = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD";
    std::string latest_yaml;
    time_t latest_time = 0;
    
    DIR* dir = opendir(map_dir.c_str());
    if (!dir) {
        std::cerr << "[PositionServer] 无法打开地图目录: " << map_dir << std::endl;
        return "";
    }
    
    // 优先检查 .current_map_file 标记文件
    std::string marker_file = map_dir + "/.current_map_file";
    std::ifstream marker(marker_file);
    if (marker.is_open()) {
        std::string yaml_filename;
        std::getline(marker, yaml_filename);
        marker.close();
        
        // 去除空格和换行
        yaml_filename.erase(0, yaml_filename.find_first_not_of(" \t\r\n"));
        yaml_filename.erase(yaml_filename.find_last_not_of(" \t\r\n") + 1);
        
        if (!yaml_filename.empty()) {
            std::string yaml_path = map_dir + "/" + yaml_filename;
            struct stat file_stat;
            if (stat(yaml_path.c_str(), &file_stat) == 0) {
                closedir(dir);
                std::cout << "[PositionServer] 从标记文件读取当前地图: " << yaml_filename << std::endl;
                return yaml_path;
            }
        }
    }
    
    // 如果没有标记文件，查找最新的yaml文件
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string filename = entry->d_name;
        // 只处理.yaml文件
        if (filename.find(".yaml") == std::string::npos && 
            filename.find(".yml") == std::string::npos) {
            continue;
        }
        
        std::string filepath = map_dir + "/" + filename;
        struct stat file_stat;
        if (stat(filepath.c_str(), &file_stat) == 0) {
            if (file_stat.st_mtime > latest_time) {
                latest_time = file_stat.st_mtime;
                latest_yaml = filepath;
            }
        }
    }
    closedir(dir);
    
    return latest_yaml;
}

/**
 * @brief 从YAML文件加载地图参数
 */
bool PositionServerNewDevice::loadMapParamsFromYAML(const std::string& yaml_file) {
    std::ifstream file(yaml_file);
    if (!file.is_open()) {
        std::cerr << "[PositionServer] 无法打开YAML文件: " << yaml_file << std::endl;
        return false;
    }
    
    std::string line;
    std::string pgm_file;
    double resolution = 0.0;
    double origin_x = 0.0, origin_y = 0.0;
    
    while (std::getline(file, line)) {
        // 去除前后空格
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        if (line.find("image:") == 0) {
            pgm_file = line.substr(6);
            pgm_file.erase(0, pgm_file.find_first_not_of(" \t"));
        } else if (line.find("resolution:") == 0) {
            std::string val = line.substr(11);
            val.erase(0, val.find_first_not_of(" \t"));
            resolution = std::stod(val);
        } else if (line.find("origin:") == 0) {
            // 解析 origin: [x, y, z] 格式
            size_t start = line.find('[');
            size_t end = line.find(']');
            if (start != std::string::npos && end != std::string::npos) {
                std::string coords = line.substr(start + 1, end - start - 1);
                std::istringstream ss(coords);
                std::string token;
                int idx = 0;
                while (std::getline(ss, token, ',')) {
                    token.erase(0, token.find_first_not_of(" \t"));
                    token.erase(token.find_last_not_of(" \t") + 1);
                    if (idx == 0) origin_x = std::stod(token);
                    else if (idx == 1) origin_y = std::stod(token);
                    idx++;
                }
            }
        }
    }
    file.close();
    
    // 处理 PGM 文件路径
    // 如果 YAML 中的路径不存在，尝试使用同名的 PGM 文件
    if (!pgm_file.empty()) {
        // 检查 YAML 中指定的路径是否存在
        struct stat file_stat;
        if (stat(pgm_file.c_str(), &file_stat) != 0) {
            // 文件不存在，尝试使用与 YAML 同名的 PGM 文件
            std::string yaml_basename = yaml_file.substr(yaml_file.find_last_of('/') + 1);
            std::string pgm_basename = yaml_basename.substr(0, yaml_basename.find_last_of('.')) + ".pgm";
            std::string yaml_dir = yaml_file.substr(0, yaml_file.find_last_of('/'));
            std::string alternative_pgm = yaml_dir + "/" + pgm_basename;
            
            std::cout << "[PositionServer] YAML中的PGM路径不存在: " << pgm_file << std::endl;
            std::cout << "[PositionServer] 尝试使用同名PGM文件: " << alternative_pgm << std::endl;
            
            if (stat(alternative_pgm.c_str(), &file_stat) == 0) {
                pgm_file = alternative_pgm;
                std::cout << "[PositionServer] 找到同名PGM文件: " << pgm_file << std::endl;
            } else {
                std::cerr << "[PositionServer] 同名PGM文件也不存在: " << alternative_pgm << std::endl;
            }
        }
    }
    
    // 从PGM文件读取地图尺寸
    int width = 0, height = 0;
    if (!pgm_file.empty() && loadMapSizeFromPGM(pgm_file, width, height)) {
        std::lock_guard<std::mutex> lock(this->positionDataMutex);
        this->mapParams.resolution = resolution;
        this->mapParams.origin_x = origin_x;
        this->mapParams.origin_y = origin_y;
        this->mapParams.width = width;
        this->mapParams.height = height;
        
        std::cout << "[PositionServer] 地图参数已更新:" << std::endl;
        std::cout << "  - 分辨率: " << resolution << " m/pixel" << std::endl;
        std::cout << "  - 原点: (" << origin_x << ", " << origin_y << ")" << std::endl;
        std::cout << "  - 尺寸: " << width << " x " << height << " pixels" << std::endl;
        return true;
    }
    
    return false;
}

/**
 * @brief 从PGM文件读取地图尺寸
 */
bool PositionServerNewDevice::loadMapSizeFromPGM(const std::string& pgm_file, int& width, int& height) {
    std::ifstream file(pgm_file, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "[PositionServer] 无法打开PGM文件: " << pgm_file << std::endl;
        return false;
    }
    
    std::string magic;
    file >> magic;
    
    if (magic != "P5" && magic != "P2") {
        std::cerr << "[PositionServer] 不支持的PGM格式: " << magic << std::endl;
        return false;
    }
    
    // 跳过注释行
    file >> std::ws;
    while (file.peek() == '#') {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        file >> std::ws;
    }
    
    // 读取宽度和高度
    file >> width >> height;
    
    if (width <= 0 || height <= 0) {
        std::cerr << "[PositionServer] 无效的PGM尺寸: " << width << "x" << height << std::endl;
        return false;
    }
    
    file.close();
    return true;
}

/**
 * @brief 启动文件监听线程
 */
void PositionServerNewDevice::startMapFileWatcher() {
    this->map_watcher_running_ = true;
    this->map_watcher_thread_ = std::thread([this]() {
        const std::string map_dir = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/PCD";
        std::string last_yaml;
        time_t last_check_time = 0;
        
        std::cout << "[PositionServer] 文件监听线程已启动，监控目录: " << map_dir << std::endl;
        
        while (this->map_watcher_running_) {
            // 每2秒检查一次
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            std::string latest_yaml = findLatestMapYAML();
            if (latest_yaml.empty()) continue;
            
            // 检查文件是否有更新
            struct stat file_stat;
            if (stat(latest_yaml.c_str(), &file_stat) == 0) {
                if (latest_yaml != last_yaml || file_stat.st_mtime > last_check_time) {
                    std::cout << "[PositionServer] 检测到新地图文件: " << latest_yaml << std::endl;
                    
                    // 等待文件写入完成（避免读取不完整的文件）
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    
                    if (loadMapParamsFromYAML(latest_yaml)) {
                        last_yaml = latest_yaml;
                        last_check_time = file_stat.st_mtime;
                        
                        // 更新地图编号
                        this->current_map_id_ = loadCurrentMapId();
                        
                        // 🔴 同时更新 currentPositionData.map_id
                        {
                            std::lock_guard<std::mutex> lock(this->positionDataMutex);
                            this->currentPositionData.map_id = this->current_map_id_;
                        }
                        
                        std::cout << "[PositionServer] 地图参数已自动更新" << std::endl;
                        std::cout << "[PositionServer] 当前地图编号: " << this->current_map_id_ << std::endl;
                    }
                }
            }
        }
        
        std::cout << "[PositionServer] 文件监听线程已停止" << std::endl;
    });
}

/**
 * @brief 停止文件监听线程
 */
void PositionServerNewDevice::stopMapFileWatcher() {
    this->map_watcher_running_ = false;
    if (this->map_watcher_thread_.joinable()) {
        this->map_watcher_thread_.join();
    }
}

/**
 * @brief 启动TF更新线程
 */
void PositionServerNewDevice::startTFUpdateThread() {
    this->tf_thread_running_ = true;
    this->tf_update_thread_ = std::thread([this]() {
        tfUpdateLoop();
    });
    std::cout << "[PositionServer] TF更新线程已启动" << std::endl;
}

/**
 * @brief 停止TF更新线程
 */
void PositionServerNewDevice::stopTFUpdateThread() {
    this->tf_thread_running_ = false;
    if (this->tf_update_thread_.joinable()) {
        this->tf_update_thread_.join();
    }
    std::cout << "[PositionServer] TF更新线程已停止" << std::endl;
}

/**
 * @brief TF更新循环 - 以10Hz频率查询TF并更新位置
 */
void PositionServerNewDevice::tfUpdateLoop() {
    ros::Rate rate(10.0);  // 10Hz更新频率
    
    std::cout << "[PositionServer] TF更新循环开始运行" << std::endl;
    
    while (this->tf_thread_running_ && ros::ok()) {
        try {
            // 等待TF变换可用
            if (this->tf_listener_.waitForTransform(this->target_frame_, this->source_frame_,
                                                     ros::Time(0), ros::Duration(0.1))) {
                // 查询TF变换
                tf::StampedTransform transform;
                this->tf_listener_.lookupTransform(this->target_frame_, this->source_frame_,
                                                   ros::Time(0), transform);
                
                // 更新位置数据
                std::lock_guard<std::mutex> lock(this->positionDataMutex);
                
                // 更新世界坐标
                this->currentPositionData.x = transform.getOrigin().x();
                this->currentPositionData.y = transform.getOrigin().y();
                
                // 计算yaw角
                tf::Quaternion q = transform.getRotation();
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                this->currentPositionData.yaw = yaw;
                
                // 计算像素坐标
                worldToPixel(this->currentPositionData.x, this->currentPositionData.y,
                            this->currentPositionData.px, this->currentPositionData.py);
                
                this->odom_received_ = true;
                
                // 定期打印位置信息
                static int count = 0;
                if (++count % 100 == 0) {
                    std::cout << "[PositionServer] TF位置更新 - 世界坐标: (" 
                              << this->currentPositionData.x << ", " << this->currentPositionData.y 
                              << "), 航向: " << this->currentPositionData.yaw << " rad ("
                              << (this->currentPositionData.yaw * 180.0 / M_PI) << " deg)"
                              << ", 像素坐标: (" << this->currentPositionData.px 
                              << ", " << this->currentPositionData.py << ")" << std::endl;
                }
            } else {
                if (!this->odom_received_) {
                    static int warn_count = 0;
                    if (++warn_count % 50 == 0) {
                        std::cout << "[PositionServer] 等待TF变换: " << this->target_frame_ 
                                  << " -> " << this->source_frame_ << std::endl;
                    }
                }
            }
        } catch (tf::TransformException& ex) {
            static int error_count = 0;
            if (++error_count % 50 == 0) {
                std::cerr << "[PositionServer] TF查询异常: " << ex.what() << std::endl;
            }
        }
        
        rate.sleep();
    }
    
    std::cout << "[PositionServer] TF更新循环已退出" << std::endl;
}

// ==================== 信号处理和主函数 ====================

/**
 * @brief 信号处理函数
 */
void signalHandler(int signum) {
    std::cout << "收到中断信号 (" << signum << ")，正在关闭位置服务..." << std::endl;
    if (g_position_server) {
        g_position_server->endWork();
    }
    ros::shutdown();
}

/**
 * @brief 主函数 - ROS节点入口
 */
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "position_server_node");
    ros::NodeHandle nh("~");
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // 读取参数
    std::string position_ip, httpserver_ip;
    int position_port, httpserver_port;
    std::string odom_topic;
    
    nh.param<std::string>("position_ip", position_ip, "192.168.1.100");
    nh.param<int>("position_port", position_port, 7070);
    nh.param<std::string>("httpserver_ip", httpserver_ip, "192.168.1.200");
    nh.param<int>("httpserver_port", httpserver_port, 8888);
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");
    
    std::cout << "=== 位置服务端配置 ===" << std::endl;
    std::cout << "位置服务端地址: " << position_ip << ":" << position_port << std::endl;
    std::cout << "HTTP服务器地址: " << httpserver_ip << ":" << httpserver_port << std::endl;
    std::cout << "里程计话题: " << odom_topic << std::endl;
    
    // 创建位置服务端实例
    g_position_server = new PositionServerNewDevice(
        position_ip.c_str(), 
        static_cast<unsigned short>(position_port),
        httpserver_ip.c_str(), 
        static_cast<unsigned short>(httpserver_port),
        &nh
    );
    
    // 设置ROS话题（如果与默认不同）
    if (odom_topic != "/Odometry") {
        g_position_server->setOdomTopic(odom_topic);
    }
    
    // 启动位置服务
    if (!g_position_server->startWork()) {
        std::cerr << "位置服务启动失败!" << std::endl;
        delete g_position_server;
        return -1;
    }
    
    std::cout << "位置服务已启动，等待ROS数据和HTTPSERVER请求..." << std::endl;
    
    // 进入ROS事件循环
    ros::spin();
    
    // 清理
    std::cout << "正在关闭位置服务..." << std::endl;
    g_position_server->endWork();
    g_position_server->watingDeviceEnding();
    delete g_position_server;
    g_position_server = nullptr;
    
    std::cout << "位置服务已关闭" << std::endl;
    return 0;
}