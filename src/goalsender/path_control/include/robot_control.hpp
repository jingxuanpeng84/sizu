#ifndef ROBOT_CONTROL_HPP
#define ROBOT_CONTROL_HPP
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
#include "path_control/include/octi_robot.hpp"


struct robotControlData {
    uint32_t command;      // 控制命令 (对应 OCTIROBOT::MANIPULATION 枚举)
    uint32_t param1;       // 参数1 (如:速度等级、标志位等)
    uint32_t param2;       // 参数2
    float param3;          // 参数3 (如:x方向速度)
    float param4;          // 参数4 (如:y方向速度或角速度)
};

class robotControl : public device
{
private:
udpTool* communicator;
std::string serverIp;
unsigned short serverPort;
std::string myIp;
unsigned short myPort;

octiRobot* robot;

//回调函数
static void robotDataCallback(device* dev);
static std::pair<std::string,int> robotResetCallback(device* dev);
static std::pair<std::string, int> robotCheckCallback(device* dev);

//UDP接收回调
static void udpRecvCallback(netWorkBase* net, void* buf, int size, sockaddr_in addr);

// 处理控制命令的辅助函数
void processControlCommand(const robotControlData& controlData);

public:
    robotControl(const char *serverIp_, unsigned short serverPort_, 
                      const char *myIp_, unsigned short myPort_, octiRobot* robot_);
    ~robotControl();
    
    bool startWork();
    bool endWork();
    void watingDeviceEnding();
    
    virtual void deviceWorkTHreadFunc(int fd) override {}
};
#endif // ROBOT_CONTROL_HPP