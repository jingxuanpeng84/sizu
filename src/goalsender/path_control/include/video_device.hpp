#ifndef VIDEO_DEVICE__HPP
#define VIDEO_DEVICE__HPP

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <opencv2/opencv.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/video/video_client.hpp>
#include "path_control/include/deviceBase.hpp"

namespace VIDEO
{

};

class videoDevice : public deviceBase
{
private:
    std::thread *ThreadHandler;
    const char *videoPath;

private:
    cv::Mat frame;

private:
    std::mutex frameAccessMutex;
    std::atomic<bool> windowShowFlag;
    std::atomic<bool> threadEndFlag;

private:
    void deviceWorkTHreadFunc(int fd) override;

public:
    bool startWork() override;
    bool endWork() override;
    void watingDeviceEnding() override;
    bool readDeviceData(void *dataBuf, unsigned int &dataSize) override;
    bool sendDeviceData(void *dataBuf, unsigned int dataSize) { return true; };
    bool isDeviceBeginWorking() override{return true;};
public:
    videoDevice(const char *videoPath_);
    ~videoDevice();
    void openWindowShow();
    void closeWindowShow();
};

#endif