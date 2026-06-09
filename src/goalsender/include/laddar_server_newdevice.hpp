#ifndef __LADDAR_SERVER_NEWDEVICE_HPP
#define __LADDAR_SERVER_NEWDEVICE_HPP

#include "/home/orangepi/slam_ws/src/goalsender/include/network/deviceBase.hpp"
#include "/home/orangepi/slam_ws/src/goalsender/include/network/tcpUdp.hpp"
#include "/home/orangepi/slam_ws/src/goalsender/include/network/frame.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>

// ROS相关头文件
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#define LINE_NUM 361  // 匹配客户端的 LADDAR_LINE_NUM
#define LADDAR_MAX_SEQ 65535

// 雷达数据结构定义（完全匹配客户端 path_control/include/laddar_device.hpp）
namespace LADDAR {
    struct laddar_data {
        double x;                    // 机器人X坐标
        double y;                    // 机器人Y坐标
        double yaw;                  // 航向角
        double dist[LINE_NUM];       // 雷达扫描距离数组 (361个点)
    };
    
    // 使用 dataStruct 别名以兼容
    typedef laddar_data dataStruct;

    struct receiveFrameStruct {
        unsigned long frameType;     // 数据帧类型（匹配客户端）
        unsigned long seq;           // 序列号
        dataStruct data;             // 实际雷达数据
    };

    struct requestFrameStruct {
        unsigned long frameType;     // 请求帧类型
        unsigned long seq;           // 请求序列号
    };
}

/**
 * @brief 雷达服务端类 - 基于newdevice框架，订阅ROS话题获取真实数据
 * 
 * 该类作为雷达端，负责：
 * 1. 订阅ROS激光雷达和里程计话题
 * 2. 接收来自ROBOT_OBJ的雷达数据请求
 * 3. 发送雷达数据给ROBOT_OBJ
 * 4. 管理雷达设备状态和心跳
 */
class LaddarServerNewDevice : public device {
private:
    // 网络配置
    std::string myIp;           // 雷达端IP
    unsigned short myPort;      // 雷达端端口
    std::string robotIp;        // 机器人端IP
    unsigned short robotPort;   // 机器人端端口
    
    // ROS相关
    ros::NodeHandle* nh_;                       // ROS节点句柄
    ros::Subscriber laser_sub_;                 // 激光雷达订阅器
    ros::Subscriber odom_sub_;                  // 里程计订阅器
    bool laser_received_;                       // 是否接收到激光数据
    bool odom_received_;                        // 是否接收到里程计数据
    std::string laser_topic_;                   // 激光雷达话题
    std::string odom_topic_;                    // 里程计话题
    
    // 雷达数据
    LADDAR::laddar_data currentLaddarData;  // 当前雷达数据
    std::mutex laddarDataMutex;             // 雷达数据互斥锁
    std::atomic<unsigned long> currentSeq;  // 当前序列号
    
    // UDP通信器
    udpTool* communicator;
    
    // ROS回调函数
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    /**
     * @brief 构造函数
     * @param myIp_ 雷达端IP地址
     * @param myPort_ 雷达端端口
     * @param robotIp_ 机器人端IP地址
     * @param robotPort_ 机器人端端口
     * @param nh ROS节点句柄指针
     */
    LaddarServerNewDevice(const char *myIp_, unsigned short myPort_,
                         const char *robotIp_, unsigned short robotPort_,
                         ros::NodeHandle* nh);
    
    /**
     * @brief 析构函数
     */
    ~LaddarServerNewDevice();
    
    // ROS配置
    void setROSTopics(const std::string& laser_topic, const std::string& odom_topic);
    bool isROSDataReady() const;
    
    // 静态回调函数
    static void laddarServerDataCallback(device* dev);
    static std::pair<std::string, int> laddarServerResetCallback(device* dev);
    static std::pair<std::string, int> laddarServerCheckCallback(device* dev);
    
    // UDP接收回调
    static void udpServerRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr);
    
    // 设备控制接口
    bool startWork();           // 启动雷达服务
    bool endWork();             // 结束雷达服务
    void watingDeviceEnding();  // 等待设备结束
    
    // 数据接口
    void updateLaddarData(const LADDAR::laddar_data& data); // 更新雷达数据
    LADDAR::laddar_data getCurrentLaddarData();             // 获取当前雷达数据
    
    // 请求处理
    void handleDataRequest(const LADDAR::requestFrameStruct& request, sockaddr_in clientAddr);
    void sendLaddarData(const LADDAR::laddar_data& data, unsigned long seq, sockaddr_in clientAddr);
};

#endif // __LADDAR_SERVER_NEWDEVICE_HPP