#ifndef OCTI_VISION__HPP
#define OCTI_VISION__HPP

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

#define VISION_MAX_SEQ (unsigned long)1000

namespace VISION
{
    enum instructionTypeEnum
    {
        REQUEST = 1,
        ALIVE = 2,
        ACK = 3,
        FINAL = 4,
        ROBOTCONTROL = 5,
    };

    struct dataStruct
    {
        enum instructionTypeEnum instruction;
        double reserveData_1;
        double reserveData_2;
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
        dataStruct data;
    };
};

class visionDevice : public deviceBase
{
private:
    std::thread *ThreadHandler;
    const char *deviceIP;
    unsigned short devicePort;
    const char *myIp;
    unsigned short myPort;
    std::atomic<bool> threadEndFlag;
    std::atomic<bool> DataUpdateFlag;
    std::mutex DataAccessMutex;
    int sockFd;

private:
    std::mutex sendDataAccessMutex;
    std::atomic<bool> waitingDataUpdateFlag;
    VISION::dataStruct WaitingSendData;

private:
    VISION::dataStruct dataBuf;

public:
    visionDevice(const char *deviceIp_, unsigned short devicePort_, const char *myIp_, unsigned short myPort_);
    ~visionDevice();
    void deviceWorkTHreadFunc(int fd) override;
    bool startWork() override;
    bool endWork() override;
    void watingDeviceEnding() override;
    bool readDeviceData(void *dataBuf, unsigned int &dataSize) override;
    bool sendDeviceData(void *dataBuf, unsigned int dataSize) override;
    bool isDeviceBeginWorking() override{return true;};
};


#endif