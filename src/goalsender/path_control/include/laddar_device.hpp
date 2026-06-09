#ifndef LADDAR_DEVICE__HPP
#define LADDAR_DEVICE__HPP

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <thread>
#include <functional> //std::bind
#include <iostream>
#include <atomic>
#include <mutex>
#include "path_control/include/deviceBase.hpp"
#include "path_control/include/tcpUdp.hpp"
#include "path_control/include/frame.hpp"
#include "path_control/include/moduleObj.hpp"

#define LADDAR_MAX_SEQ (unsigned long)1000
#define LADDAR_LINE_NUM 361
namespace LADDAR
{
    struct dataStruct
    {
        double x;
        double y;
        double yaw;
        double dist[LADDAR_LINE_NUM];
    };
    struct receiveFrameStruct
    {
        unsigned long frameType;
        unsigned long seq;
        dataStruct data;
    };
    struct requestFrameStruct
    {
        unsigned long frameType;
        unsigned long seq;
    };
};

class laddarDevice : public device
{
private:
//数据缓冲
    std::atomic<bool> DataUpdateFlag;
    std::atomic<bool> laddarBeginFlag;
    std::mutex DataAccessMutex;
    LADDAR::dataStruct dataBuf;
//通信器
    udpTool* communicator;
    std::string deviceIP;
    unsigned short devicePort;
    std::string myIp;
    unsigned short myPort;

//请求线程
std::thread* requestThread;
std::atomic<bool> requestThreadRunning;
void requestThreadFunc();    
//回调函数
    static void laddarDataCallback(device* dev);
    static std::pair<std::string, int> laddarResetCallback(device* dev);
    static std::pair<std::string, int> laddarCheckCallback(device* dev);

//UDP接收回调
    static void udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr);
    // std::thread *ThreadHandler;
    
    // std::atomic<bool> threadEndFlag;

public:
    laddarDevice(const char *deviceIp_, unsigned short devicePort_, const char *myIp_, unsigned short myPort_);
    ~laddarDevice();
    bool startWork();
    bool endWork();
    void watingDeviceEnding();
    bool readDeviceData(void *dataBuf, unsigned int &dataSize);
    bool sendDeviceData(void *dataBuf, unsigned int dataSize);
    bool isDeviceBeginWorking();
    virtual void deviceWorkTHreadFunc(int fd) override {

}
};

#endif