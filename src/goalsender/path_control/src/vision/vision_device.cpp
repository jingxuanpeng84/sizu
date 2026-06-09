#include "path_control/include/vision_device.hpp"

visionDevice::visionDevice(const char *deviceIp_, unsigned short devicePort_, const char *myIp_, unsigned short myPort_)
{
    this->threadEndFlag = false;
    this->DataUpdateFlag = false;
    this->waitingDataUpdateFlag = false;
    this->deviceIP = deviceIp_;
    this->devicePort = devicePort_;
    this->myIp = myIp_;
    this->myPort = myPort_;
    this->ThreadHandler = NULL;
    this->sockFd = -1;
    // std::cout << "Vision ip = " << this->deviceIP << std::endl;
}

visionDevice::~visionDevice()
{
    if (this->ThreadHandler != NULL)
    {
        if (this->ThreadHandler->joinable())
        {
            this->ThreadHandler->join();
            std::cout << "Device close success\n";
        }
    }
}

bool visionDevice::endWork()
{
    this->threadEndFlag = true;
    return true;
}

bool visionDevice::startWork()
{
    this->threadEndFlag = false;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::cout << "sock error\n";
        return false;
    }
    this->sockFd = fd;
    sockaddr_in mySockAddrIn;
    memset(&mySockAddrIn, 0, sizeof(mySockAddrIn));
    mySockAddrIn.sin_family = AF_INET;
    mySockAddrIn.sin_port = htons(this->myPort);
    mySockAddrIn.sin_addr.s_addr = inet_addr(this->myIp);
    int ret = bind(fd, (sockaddr *)&mySockAddrIn, sizeof(mySockAddrIn));
    if (ret < 0)
    {
        std::cout << "bind error\n";
        close(fd);
        this->sockFd = -1;
        return false;
    }
    this->ThreadHandler = new std::thread(std::bind(&visionDevice::deviceWorkTHreadFunc, this, fd));
    std::cout << "Vision Device start success!\n";
    return true;
}

void visionDevice::watingDeviceEnding()
{
    if (this->ThreadHandler != NULL)
    {
        this->ThreadHandler->join();
        this->ThreadHandler = NULL;
    }
    std::cout << "Vision Device finished\n";
}

void visionDevice::deviceWorkTHreadFunc(int fd)
{
    std::cout << "device thread begin!\n";
    // 先发数据请求
    unsigned long seq = 0;
    sockaddr_in server;
    sockaddr_in send_c;
    socklen_t size = sizeof(send_c);
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(this->deviceIP);
    server.sin_port = htons(this->devicePort);
    VISION::requestFrameStruct send_data;
    VISION::receiveFrameStruct receive_data;
    ssize_t num = 0;
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    do
    {
        if (this->threadEndFlag == true)
        {
            close(fd);
            std::cout << "Vision Device close success\n";
            pthread_exit(NULL);
        }
        if (this->waitingDataUpdateFlag == true)
        {
            send_data.frameType = 1;
            send_data.seq = seq;
            this->sendDataAccessMutex.lock();
            send_data.data = this->WaitingSendData;
            sendto(fd, &send_data, sizeof(send_data), 0, (sockaddr *)&server, sizeof(server));
            this->waitingDataUpdateFlag = false;
            this->sendDataAccessMutex.unlock();
        }
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        //
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;
        //

        int result = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (result < 0)
        {
            perror("select failed");
            close(fd);
            exit(EXIT_FAILURE);
        }
        else if (result == 0)
        {
            continue;
        }
        // receive vision data
        num = recvfrom(fd, &receive_data, sizeof(receive_data), 0, (sockaddr *)&send_c, &size);
        if (num == sizeof(VISION::receiveFrameStruct) && receive_data.frameType == 2 ) //&& receive_data.seq == seq
        {
            seq = (seq + 1) % VISION_MAX_SEQ;
            this->DataAccessMutex.lock();
            this->dataBuf = receive_data.data;
            this->DataUpdateFlag = true;
            this->DataAccessMutex.unlock();
            this->waitingDataUpdateFlag = false;
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }
        else if (receive_data.seq < seq)
        {
            // std::cout << "receive req error recSeq = " << receive_data.seq << " Seq = " << seq << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }
    } while (true);
}

bool visionDevice::readDeviceData(void *dataBuf, unsigned int &dataSize)
{
    unsigned int temptTimes = 0;
    while (true)
    {
        if (this->DataUpdateFlag == true)
        {
            this->DataAccessMutex.lock();
            memcpy(dataBuf, &(this->dataBuf), sizeof(this->dataBuf));
            this->DataUpdateFlag = false;
            this->DataAccessMutex.unlock();
            return true;
        }
        else
        {
            temptTimes++;
            if (temptTimes >= 20)
            {
                std::cout << "vision device may disconnect or data loss!\n";
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }
    return true;
}

bool visionDevice::sendDeviceData(void *dataBuf, unsigned int dataSize)
{
    if (dataSize != sizeof(VISION::dataStruct))
    {
        return false;
    }
    this->sendDataAccessMutex.lock();
    memcpy(&(this->WaitingSendData), dataBuf, sizeof(VISION::dataStruct));
    this->sendDataAccessMutex.unlock();
    this->waitingDataUpdateFlag = true;
    return true;
}