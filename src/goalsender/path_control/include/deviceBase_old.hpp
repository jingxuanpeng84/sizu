#ifndef DEVICE_BASE_HPP
#define DEVICE_BASE_HPP

class deviceBase
{
public:


public:
    virtual bool readDeviceData(void *dataBuf, unsigned int &dataSize) = 0;
    virtual bool sendDeviceData(void *dataBuf, unsigned int dataSize) = 0;
    virtual void deviceWorkTHreadFunc(int fd) = 0;
    virtual bool startWork() = 0;
    virtual bool isDeviceBeginWorking() = 0;
    virtual bool endWork() = 0;
    virtual void watingDeviceEnding() = 0;

    deviceBase(){};
    virtual ~deviceBase(){};
};

#endif