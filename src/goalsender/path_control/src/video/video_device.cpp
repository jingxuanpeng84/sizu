#include "path_control/include/video_device.hpp"

videoDevice::videoDevice(const char *videoPath_)
{
    this->windowShowFlag = false;
    this->threadEndFlag = false;
    if (videoPath_ != NULL)
    {
        this->videoPath = videoPath_;
    }
    else
    {
        perror("ERROR : videoPath is Null!");
    }
}

videoDevice::~videoDevice()
{
    this->endWork();
    this->watingDeviceEnding();
}

/*
* @param dataBuf -> (cv::Mat *)
*/
bool videoDevice::readDeviceData(void *dataBuf, unsigned int &dataSize)
{
    this->frameAccessMutex.lock();
    if (this->frame.empty())
    {
        this->frameAccessMutex.unlock();
        return false;
    }
    *((cv::Mat *)dataBuf) = this->frame.clone();
    // dataSize = this->frame.total() * this->frame.elemSize();
    this->frameAccessMutex.unlock();
    return true;
}

void videoDevice::openWindowShow()
{
    this->windowShowFlag = true;
}

void videoDevice::closeWindowShow()
{
    this->windowShowFlag = false;
}

void videoDevice::deviceWorkTHreadFunc(int fd)
{
    cv::VideoCapture *videoCaptureHandle = NULL;
    unsigned int windowOpenedFlag = 0;
    if (this->videoPath != NULL)
    {
        std::cout << "Video path = " << this->videoPath << std::endl;
        videoCaptureHandle = new cv::VideoCapture(this->videoPath, cv::CAP_GSTREAMER);
        std::cout << "open vide0\n";
    }
    else
    {
        videoCaptureHandle = new cv::VideoCapture(fd);
    }
    if (!videoCaptureHandle->isOpened())
    {
        std::cerr << "Unable to open camera" << std::endl;
        pthread_exit(NULL);
    }
    while (this->threadEndFlag != true)
    {
        this->frameAccessMutex.lock();
        videoCaptureHandle->read(this->frame);
        if (this->windowShowFlag)
        {
            windowOpenedFlag = 1;
            cv::imshow("Robot Video", this->frame);
            cv::waitKey(1);
        }
        else
        {
            if (windowOpenedFlag == 1)
            {
                std::cout << "des\n";
                cv::destroyWindow("Robot Video");
                cv::waitKey(1);
                windowOpenedFlag = 0;
            }
        }
        this->frameAccessMutex.unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    videoCaptureHandle->release();
    cv::destroyAllWindows();
    std::cout << "thread exit\n";
    pthread_exit(NULL);
}

bool videoDevice::startWork()
{
    this->threadEndFlag = false;
    this->ThreadHandler = new std::thread(std::bind(&videoDevice::deviceWorkTHreadFunc, this, 0));
    std::cout << "Video Thread start success!\n";
    return true;
}

void videoDevice::watingDeviceEnding()
{
    if (this->ThreadHandler != NULL)
    {
        this->ThreadHandler->join();
        this->ThreadHandler = NULL;
    }
    std::cout << "Device finished\n";
}

bool videoDevice::endWork()
{
    this->threadEndFlag = true;
    return true;
}