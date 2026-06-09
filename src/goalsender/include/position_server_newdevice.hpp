#ifndef __POSITION_SERVER_NEWDEVICE_HPP
#define __POSITION_SERVER_NEWDEVICE_HPP

#include "/home/orangepi/slam_ws/src/goalsender/include/network/deviceBase.hpp"
#include "/home/orangepi/slam_ws/src/goalsender/include/network/tcpUdp.hpp"
#include "/home/orangepi/slam_ws/src/goalsender/include/network/frame.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <dirent.h>
#include <sys/stat.h>
#include <fstream>
#include <sstream>
#include <limits>

// ROS相关头文件
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#define POSITION_MAX_SEQ 65535

// 位置数据结构定义
namespace POSITION {
    // 位置数据
    struct position_data {
        double x;                    // 机器人X坐标（世界坐标）
        double y;                    // 机器人Y坐标（世界坐标）
        double yaw;                  // 航向角
        int px;                      // 像素X坐标
        int py;                      // 像素Y坐标
        int map_id;                  // 地图编号
    };

    // 位置响应帧结构
    struct receiveFrameStruct {
        unsigned long frame_type;    // 数据帧类型
        unsigned long seq;           // 序列号
        position_data data;          // 实际位置数据
    };

    // 位置请求帧结构
    struct requestFrameStruct {
        unsigned long frameType;     // 请求帧类型
        unsigned long seq;           // 请求序列号
    };
    
    // 地图参数结构
    struct MapParams {
        double resolution;           // 地图分辨率（米/像素）
        double origin_x;             // 地图原点X坐标
        double origin_y;             // 地图原点Y坐标
        int width;                   // 地图宽度（像素）
        int height;                  // 地图高度（像素）
    };
}

/**
 * @brief 位置服务端类 - 基于newdevice框架，订阅ROS话题获取位置数据
 * 
 * 该类作为雷达端，负责：
 * 1. 订阅ROS里程计话题获取位置信息
 * 2. 接收来自HTTPSERVER_OBJ的位置数据请求
 * 3. 发送位置数据给HTTPSERVER_OBJ
 * 4. 管理设备状态和心跳
 */
class PositionServerNewDevice : public device {
private:
    // 网络配置
    std::string myIp;           // 位置服务端IP
    unsigned short myPort;      // 位置服务端端口
    std::string httpServerIp;   // HTTP服务器端IP
    unsigned short httpServerPort; // HTTP服务器端端口
    
    // ROS相关
    ros::NodeHandle* nh_;                       // ROS节点句柄
    ros::Subscriber odom_sub_;                  // 里程计订阅器
    tf::TransformListener tf_listener_;         // TF监听器
    bool odom_received_;                        // 是否接收到里程计数据
    std::string odom_topic_;                    // 里程计话题
    std::string source_frame_;                  // 源坐标系
    std::string target_frame_;                  // 目标坐标系
    bool use_tf_;                               // 是否使用TF（true）还是Odometry（false）
    std::thread tf_update_thread_;              // TF更新线程
    std::atomic<bool> tf_thread_running_;       // TF线程运行标志
    
    // 位置数据
    POSITION::position_data currentPositionData;  // 当前位置数据
    std::mutex positionDataMutex;                 // 位置数据互斥锁
    std::atomic<unsigned long> currentSeq;        // 当前序列号
    
    // 地图参数
    POSITION::MapParams mapParams;                // 地图参数
    int current_map_id_;                          // 当前地图编号
    
    // 文件监听
    std::thread map_watcher_thread_;              // 文件监听线程
    std::atomic<bool> map_watcher_running_;       // 监听线程运行标志
    
    // UDP通信器
    udpTool* communicator;
    
    // 辅助函数
    void worldToPixel(double world_x, double world_y, int& pixel_x, int& pixel_y);
    bool loadMapParamsFromYAML(const std::string& yaml_file);  // 从YAML加载地图参数
    bool loadMapSizeFromPGM(const std::string& pgm_file, int& width, int& height);  // 从PGM加载地图尺寸
    std::string findLatestMapYAML();                           // 查找最新的YAML文件
    int loadCurrentMapId();                                    // 加载当前地图编号
    void startMapFileWatcher();                                // 启动文件监听
    void stopMapFileWatcher();                                 // 停止文件监听
    void startTFUpdateThread();                                // 启动TF更新线程
    void stopTFUpdateThread();                                 // 停止TF更新线程
    void tfUpdateLoop();                                       // TF更新循环
    
    // ROS回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    /**
     * @brief 构造函数
     * @param myIp_ 位置服务端IP地址
     * @param myPort_ 位置服务端端口
     * @param httpServerIp_ HTTP服务器端IP地址
     * @param httpServerPort_ HTTP服务器端端口
     * @param nh ROS节点句柄指针（必须提供）
     */
    PositionServerNewDevice(const char *myIp_, unsigned short myPort_,
                           const char *httpServerIp_, unsigned short httpServerPort_,
                           ros::NodeHandle* nh);
    
    /**
     * @brief 析构函数
     */
    ~PositionServerNewDevice();
    
    // ROS配置
    void setOdomTopic(const std::string& odom_topic);
    bool isPositionDataReady() const;
    
    // 静态回调函数
    static void positionServerDataCallback(device* dev);
    static std::pair<std::string, int> positionServerResetCallback(device* dev);
    static std::pair<std::string, int> positionServerCheckCallback(device* dev);
    
    // UDP接收回调
    static void udpServerRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr);
    
    // 设备控制接口
    bool startWork();           // 启动位置服务
    bool endWork();             // 结束位置服务
    void watingDeviceEnding();  // 等待设备结束
    
    // 数据接口
    void updatePositionData(const POSITION::position_data& data); // 更新位置数据
    POSITION::position_data getCurrentPositionData();             // 获取当前位置数据
    
    // 请求处理
    void handleInquiryRequest(const sockaddr_in& clientAddr);  // 处理INQUIRY请求
    void handleDataRequest(const POSITION::requestFrameStruct& request, sockaddr_in clientAddr);
    void sendPositionData(const POSITION::position_data& data, unsigned long seq, sockaddr_in clientAddr);
};

#endif // __POSITION_SERVER_NEWDEVICE_HPP
