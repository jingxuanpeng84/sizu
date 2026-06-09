#include "tf_publish_node.hpp"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <cstring>

/**
 * @brief 计算数组指定范围的平均值（忽略无穷大值）
 * @tparam T 数组元素类型
 * @param arry 输入数组
 * @param start 起始索引
 * @param end 结束索引
 * @return 指定范围的平均值，如果全部是无穷大返回 infinity
 */
template<typename T>
float ArrayGetAverage(const std::vector<T>& arry, int start, int end){
    int array_size = arry.size();
    float total_value = 0;
    int filter_flag = 0;

    if(start > (array_size - 1) || end > (array_size - 1) || start > end) 
        return 0;

    for(int i = start; i <= end; i++){
        if(std::isinf(arry[i])){
            continue;
        }
        total_value += arry[i];
        filter_flag++;
    }

    if (filter_flag == 0) {
        return std::numeric_limits<float>::infinity();
    }

    return total_value / filter_flag;
}

/**
 * @brief 获取当前时间戳字符串，格式：YYYY-MM-DD HH:MM:SS.mmm
 * @return 时间戳字符串
 */
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();

    return ss.str();
}

/**
 * @brief 构造函数，初始化ROS参数、TF发布器、LaserScan订阅器
 */
TfPublish::TfPublish()
    : device("TfPublishServer", nullptr, DeviceCallbackTable(
        TfPublish::tfPublishDataCallback,
        TfPublish::tfPublishResetCallback,
        TfPublish::tfPublishCheckCallback
    ), HTTPSERVER_OBJ)
{
    // ROS参数读取
    nh.param<std::string>("pub_topic",pub_topic,"/robot_position");
    nh.param<std::string>("source_frame",source_frame,"/robot");
    nh.param<std::string>("target_frame",target_frame,"/robot_body");
    nh.param<bool>("socket_print_en",socket_print_en,true);
    nh.param<double>("publish_tf_rate",publish_tf_rate,10.0);

    // 网络参数读取（使用私有命名空间，支持 launch 文件中节点级别的参数覆盖）
    ros::NodeHandle nh_private("~");
    std::string server_addr_tmp, client_addr_tmp;
    int server_port_tmp, client_port_tmp;
    // 先从私有命名空间读，找不到再从全局读
    nh_private.param<std::string>("server_addr", server_addr_tmp, "127.0.0.1");
    nh_private.param<int>("server_PORT", server_port_tmp, 6001);
    nh_private.param<std::string>("client_addr", client_addr_tmp, "127.0.0.1");
    nh_private.param<int>("client_PORT", client_port_tmp, 6002);
    // 如果私有参数是默认值，再尝试全局参数
    if (server_addr_tmp == "127.0.0.1") nh.param<std::string>("server_addr", server_addr_tmp, "127.0.0.1");
    if (server_port_tmp == 6001) nh.param<int>("server_PORT", server_port_tmp, 6001);
    if (client_addr_tmp == "127.0.0.1") nh.param<std::string>("client_addr", client_addr_tmp, "127.0.0.1");
    if (client_port_tmp == 6002) nh.param<int>("client_PORT", client_port_tmp, 6002);

    this->myIp = server_addr_tmp;
    this->myPort = static_cast<unsigned short>(server_port_tmp);
    this->robotIp = client_addr_tmp;
    this->robotPort = static_cast<unsigned short>(client_port_tmp);

    nh.param<bool>("socket_en",socket_en,true);
    nh.param<bool>("http_en",http_en,true);

    // 初始化成员变量
    this->currentSeq = 0;
    this->communicator = nullptr;
    memset(&this->locationData_send, 0, sizeof(this->locationData_send));
    memset(&this->location_with_2dLidar, 0, sizeof(this->location_with_2dLidar));

    // ROS订阅和发布
    tf_pub = nh.advertise<std_msgs::Float32MultiArray>(pub_topic,100);
    nh.param<std::string>("sub_LaserScan_topic",sub_LaserScan_topic,"/mid360/scan");
    sub_LaserScan = nh.subscribe(sub_LaserScan_topic,1000,&TfPublish::LaserscanCallback,this);

    ROS_INFO("[TfPublish] TF发布服务端已创建");
    ROS_INFO("  - 本机地址: %s:%d", this->myIp.c_str(), this->myPort);
    ROS_INFO("  - 目标地址: %s:%d", this->robotIp.c_str(), this->robotPort);
}

/**
 * @brief 析构函数
 */
TfPublish::~TfPublish() {
    endWork();
    watingDeviceEnding();
}

/**
 * @brief LaserScan回调函数，计算左右侧平均距离并保存扫描数据
 * @param msg LaserScan消息指针
 */
void TfPublish::LaserscanCallback(const sensor_msgs::LaserScan::Ptr &msg){
    std::vector<float> angle_range(std::begin(msg->ranges),std::end(msg->ranges));
    left_distance = ArrayGetAverage<float>(angle_range, 265, 275);
    right_distance = ArrayGetAverage<float>(angle_range, 85, 95);

    int scan_size = (int)angle_range.size();

    // 与 gotest 的处理逻辑对齐：
    // gotest 里 d_lidar[i] = laddarData.dist[359 - i]，取前 360 个点
    // LaserScan 721 点覆盖 -180° 到 +180°，步长约 0.5°
    // 取 -90°(idx≈180) 到 +90°(idx≈540) 共 360 个点，按顺序存入 dist[0..359]
    int half = scan_size / 2;
    for (int i = 0; i < 360 && i < scan_size; i++) {
        int src_idx = half - 180 + i;
        if (src_idx < 0) src_idx = 0;
        if (src_idx >= scan_size) src_idx = scan_size - 1;
        float v = angle_range[src_idx];
        float val = (std::isfinite(v) && v >= msg->range_min && v <= msg->range_max) ? v : msg->range_max;
        locationData_send.data.dist[i] = val;
    }
    locationData_send.data.dist[360] = msg->range_max;

    std::lock_guard<std::mutex> lock(laddarDataMutex);
    for (int i = 0; i <= 360; i++) {
        location_with_2dLidar.dist[i] = locationData_send.data.dist[i];
    }
}

/**
 * @brief 获取socket启用状态
 * @return true表示启用
 */
bool TfPublish::socketEn_get() const{
    return socket_en;
}

/**
 * @brief 获取HTTP启用状态
 * @return true表示启用
 */
bool TfPublish::httpEn_get() const{
    return http_en;
}

// ==================== 静态回调函数 ====================

/**
 * @brief 数据处理回调
 */
void TfPublish::tfPublishDataCallback(device* dev) {
    auto data = dev->frontData();
    if (data.first == nullptr) return;
    
    struct frame* frameData = (struct frame*)data.first;
    
    // 处理心跳帧中的重启命令
    if (frameData->type == FRAME_TYPE_HEARTBEAT) {
        struct heartdancefra* hbFrame = (struct heartdancefra*)frameData->data;
        if (hbFrame->type == HEARTBEAT_COMMAND_RESTART) {
            ROS_INFO("[TfPublish] 收到重启命令");
            dev->ResetModule();
        }
    }
    
    dev->popData();
}

/**
 * @brief 重启回调
 */
std::pair<std::string, int> TfPublish::tfPublishResetCallback(device* dev) {
    ROS_INFO("[TfPublish] 执行重启...");
    
    TfPublish* tfPublish = static_cast<TfPublish*>(dev);
    if (tfPublish) {
        tfPublish->currentSeq = 0;
        std::lock_guard<std::mutex> lock(tfPublish->laddarDataMutex);
        memset(&tfPublish->locationData_send, 0, sizeof(tfPublish->locationData_send));
    }
    
    return std::make_pair("TF发布服务重启成功", 0);
}

/**
 * @brief 状态检查回调
 */
std::pair<std::string, int> TfPublish::tfPublishCheckCallback(device* dev) {
    dev->feedWatchdog();
    return std::make_pair("TF发布服务正常", 0);
}

/**
 * @brief UDP接收回调
 */
void TfPublish::udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr) {
    if (!buf || size <= 0) return;
    
    struct frame* frameData = (struct frame*)buf;
    device* dev = (device*)net->getPrivateData();
    TfPublish* tfPublish = static_cast<TfPublish*>(dev);
    
    if (!tfPublish) return;
    
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
        if (frameData->len_data >= sizeof(laddar_req_frame)) {
            laddar_req_frame* request = (laddar_req_frame*)frameData->data;
            
            ROS_INFO_THROTTLE(5.0, "[TfPublish] 收到ROBOT请求，seq: %lu from %s:%d",
                              request->seq, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
            
            tfPublish->handleDataRequest(*request);
        }
        
        if (dev) dev->pushData((char*)buf, size);
    }
}

// ==================== 数据处理函数 ====================

/**
 * @brief 处理数据请求
 */
void TfPublish::handleDataRequest(const laddar_req_frame& request) {
    laddar_data currentData;
    {
        std::lock_guard<std::mutex> lock(this->laddarDataMutex);
        currentData = this->location_with_2dLidar;
    }
    
    sendLaddarData(currentData, request.seq);
}

/**
 * @brief 发送雷达数据
 */
void TfPublish::sendLaddarData(const laddar_data& data, unsigned long seq) {
    laddar_receive_frame response;
    response.frame_type = 1;
    response.seq = seq;
    response.data = data;
    
    if (this->communicator) {
        this->communicator->sendDataByMap(&response, sizeof(response), ROBOT_OBJ, 4);
    }
    
    static int sendCount = 0;
    sendCount++;
    if (sendCount % 50 == 0) {
        std::cout << "[TfPublish] 第" << sendCount << "帧 pos=("
                  << data.x << ", " << data.y << ") yaw=" << data.yaw << "\ndist: ";
        for (int i = 0; i < LINE_NUM; i++) {
            std::cout << data.dist[i];
            if (i < LINE_NUM - 1) std::cout << " ";
        }
        std::cout << std::endl;
    }
}

// ==================== 网络服务函数 ====================

/**
 * @brief 启动网络服务
 */
bool TfPublish::startWork() {
    // 创建 UDP 通信器
    struct ipPort localAddr = {this->myIp, this->myPort};
    this->communicator = new udpTool(localAddr, MODULE_TYPE::LADDAR_OBJ);

    if (this->communicator->createNet(FRAME_MAX_SIZE) != SUCCESS) {
        ROS_ERROR("[TfPublish] sock error");
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 注册ROBOT_OBJ地址映射
    this->communicator->registerDestIpMap(this->robotIp, this->robotPort, ROBOT_OBJ);
    this->communicator->setPrivateData(this);
    
    // 启动接收线程
    if (this->communicator->startRecvThread(TfPublish::udpRecvCallback) != SUCCESS) {
        ROS_ERROR("[TfPublish] thread start error");
        delete this->communicator;
        this->communicator = nullptr;
        return false;
    }
    
    // 启动心跳
    this->startHeartbeat();
    
    ROS_INFO("[TfPublish] TF发布服务启动成功!");
    ROS_INFO("  - 监听端口: %d", this->myPort);
    ROS_INFO("  - 目标ROBOT: %s:%d", this->robotIp.c_str(), this->robotPort);
    
    return true;
}

/**
 * @brief 停止网络服务
 */
bool TfPublish::endWork() {
    // 停止心跳
    this->stopHeartbeat();
    
    // 停止接收线程
    if (this->communicator) {
        this->communicator->endRecvThread();
    }
    
    return true;
}

/**
 * @brief 等待服务结束
 */
void TfPublish::watingDeviceEnding() {
    if (this->communicator) {
        this->communicator->destroryNet();
        delete this->communicator;
        this->communicator = nullptr;
    }
    
    ROS_INFO("[TfPublish] TF发布服务结束");
}

// ==================== 原有功能函数 ====================

/**
 * @brief 运行TF监听和数据发布线程
 */
void TfPublish::run(){
    ros::Rate rate(publish_tf_rate);
    memset(&nav_data, 0,sizeof(nav_data));
    memset(&rel_coords_data, 0,sizeof(rel_coords_data));

    ROS_WARN("<--------Transform is running-------->");

    while (ros::ok()){
        try{
            tf::StampedTransform TF;
            tf::Quaternion rotation;

            /** 等待TF变换 */
            listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(5.0));
            if (!listener.canTransform(target_frame, source_frame, ros::Time(0))) {
                ROS_WARN("Failed to get transform from %s to %s", source_frame.c_str(), target_frame.c_str());
                continue;
            }

            /** 获取变换信息 */
            listener.lookupTransform(target_frame,source_frame,ros::Time(0),TF);
            rotation = TF.getRotation();
            tf::Matrix3x3 rotation_matrix(rotation);
            rotation_matrix.getRPY(roll,pitch,yaw);

            /** 更新共享导航数据和雷达数据 */
            {
                std::lock_guard<std::mutex> lock(laddarDataMutex);
                
                // 更新导航数据
                nav_data = {0,float(TF.getOrigin().x()),float(TF.getOrigin().y()),float(TF.getOrigin().z()),float(yaw),left_distance,right_distance};

                // 更新雷达位置数据（只更新 x/y/yaw，dist 由 LaserscanCallback 填充）
                location_with_2dLidar.x = TF.getOrigin().x();
                location_with_2dLidar.y = TF.getOrigin().y();
                location_with_2dLidar.yaw = yaw;

                // 更新待发送数据（只更新 x/y/yaw，dist 由 LaserscanCallback 填充，不覆盖）
                locationData_send.frame_type = 0;
                locationData_send.data.x   = location_with_2dLidar.x;
                locationData_send.data.y   = location_with_2dLidar.y;
                locationData_send.data.yaw = location_with_2dLidar.yaw;

                rel_coords_data[0]=TF.getOrigin().x();
                rel_coords_data[1]=TF.getOrigin().y();
                rel_coords_data[2]=TF.getOrigin().z();
                rel_coords_data[3]=roll;
                rel_coords_data[4]=pitch;
                rel_coords_data[5]=yaw;
            }

            /** 可选日志输出 */
            if(socket_print_en){
                ROS_WARN("Got transform from %s to %s: [%f, %f, %f, %f, %f]",
                    source_frame.c_str(),target_frame.c_str(),
                    TF.getOrigin().x(),
                    TF.getOrigin().y(),
                    yaw,
                    left_distance,
                    right_distance);                 
            } 
        }
        catch(tf::LookupException &e){
            ROS_WARN("Failed to get transform");
        }

        ros::spinOnce();
        rate.sleep();
    }  
}
