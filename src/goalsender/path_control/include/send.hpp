#ifndef SEND__HPP
#define SEND__HPP

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <iostream>
#include <atomic>
#include "path_control/include/deviceBase.hpp"
#include "path_control/include/tcpUdp.hpp"
#include "path_control/include/frame.hpp"
#include "path_control/include/moduleObj.hpp"


// ============================================
// 路径点数据结构
// ============================================
struct pathPointData {
    uint32_t sequence;      // 路径点序号
    uint32_t keyPointType;  // 关键点类型
    float x;                // x坐标
    float y;                // y坐标
};

struct timestampData {
    uint32_t flag1;         // 标志位1
    uint32_t flag2;         // 标志位2
    float timestamp1;       // 时间戳1
    float timestamp2;       // 时间戳2
};

class sendDevice : public device
{
private:

//通信器
    udpTool* communicator;
    std::string deviceIP;
    unsigned short devicePort;
    std::string myIp;
    unsigned short myPort;

//回调函数
    static void sendDataCallback(device* dev);
    static std::pair<std::string, int> sendResetCallback(device* dev);
    static std::pair<std::string, int> sendCheckCallback(device* dev);

//UDP接收回调
    static void udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr);
    // std::thread *ThreadHandler;
    
    // std::atomic<bool> threadEndFlag;

public:
    sendDevice(const char *deviceIp_, unsigned short devicePort_, const char *myIp_, unsigned short myPort_);
    ~sendDevice();
    bool startWork();
    bool endWork();
    void watingDeviceEnding();
    //发送路径点
    bool sendPathPoint(uint32_t sequence, uint32_t keyPointType, float x, float y);
    //发送时间戳
    bool sendTimestamp(uint32_t flag1, uint32_t flag2, float timestamp1, float timestamp2);
    virtual void deviceWorkTHreadFunc(int fd) override {}
};

#endif