#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cstdlib>
#include "/home/orangepi/slam_ws/src/goalsender/include/network/tcpUdp.hpp"


/* -------------UDP----------- */
udpTool::udpTool(struct ipPort _sourceIpPort, MODULE_TYPE moduleId) : netWorkBase(_sourceIpPort, moduleId)
{
    this->threadStopFlag_ = true;
};

udpTool::~udpTool()
{
    this->endRecvThread();
    this->destroryNet();
}

ERROR_NUM udpTool::createNet(int _bufSize)
{
    // udp
    this->fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(this->fd_ < 0)
    {
        printError("Socket create failed!\n");
        return SOCK_CREATE_FAIL;
    }
    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_port = htons(this->sourceIpPort_.port);
    if(!this->sourceIpPort_.ip.empty())
    {
        if (inet_pton(AF_INET, this->sourceIpPort_.ip.c_str(), &address.sin_addr) <= 0) {
            printError("Ip attach failed!\n");
            return SOCK_CREATE_FAIL;
        }
    }
    else {
        /* 本机可用ip localhost */
        address.sin_addr.s_addr = INADDR_ANY;
    }
    if (bind(this->fd_, (sockaddr*)&address, sizeof(address)) < 0) {
        printError("Socket bind failed!\n");
        return SCOKET_BIND_FAIL;
    }
    this->bufSize_ = _bufSize;
    this->dataBuf_ = new char[_bufSize];
    if(this->dataBuf_ == nullptr)
    {
        printError("alloc socket data buf failed!\n");
        return SCOKET_BIND_FAIL;
    }
    return SUCCESS;
}

void udpTool::serverWorkThread()
{
    // UDP
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char* socketBuf = new char[this->bufSize_];
    if(!socketBuf)
    {
        printError("Thread buf alloc failed\n");
        return;
    }
    while(!this->threadStopFlag_)
    {
        // data receive
        // 最好写成非阻塞，否则getData也会被阻塞，因为mutex
        // 或者改成现在这个样子，将recvfrom阻塞的放在mutex外面（但是得要设置一个缓冲区）
        int bytes_read = recvfrom(this->fd_, socketBuf, this->bufSize_, 0, (sockaddr*)&client_addr, &addr_len);
        if(bytes_read > 0)
        {
            this->dataAccess_.lock();
            memcpy(this->dataBuf_, socketBuf, bytes_read);
            this->dataSize = bytes_read;
            this->dataAccess_.unlock();
            if(this->callBack_)
            {
                // sequence: no need to lock
                this->callBack_(this, this->dataBuf_, bytes_read, client_addr);
            }
        }
        else if(bytes_read < 0)
        {
            // socket 被关闭或出错，退出线程
            break;
        }
    }
    delete[] socketBuf;
    // printLog(DEBUG, "UDP Server Thread exit\n");
}

// void udpTool::clientWorkThread()
// {
//     // UDP
//     sockaddr_in client_addr;
//     socklen_t addr_len = sizeof(client_addr);
//     char* socketBuf = new char[this->bufSize_];
//     if(!socketBuf)
//     {
//         printError("Thread buf alloc failed\n");
//         return;
//     }
//     // print(DEBUG, "client Thread start\n");
//     while(!this->threadStopFlag_)
//     {
//         // data receive
//         // 最好写成非阻塞，否则getData也会被阻塞，因为mutex
//         // 或者改成现在这个样子，将recvfrom阻塞的放在mutex外面（但是得要设置一个缓冲区）
//         int bytes_read = recvfrom(this->fd_, socketBuf, this->bufSize_, 0, (sockaddr*)&client_addr, &addr_len);
//         if(bytes_read > 0)
//         {
//             this->dataAccess_.lock();
//             memcpy(this->dataBuf_, socketBuf, bytes_read);
//             this->dataSize = bytes_read;
//             this->dataAccess_.unlock();
//             if(this->callBack_)
//             {
//                 // sequence: no need to lock
//                 this->callBack_(this, this->dataBuf_, bytes_read, client_addr);
//             }
//         }   
//     }
//     delete[] socketBuf;
// }

ERROR_NUM udpTool::startRecvThread(callBackFunc _callBack)
{
    if(this->fd_ < 0)
    {
        printError("Socket not created, must call createNet() to create socket before startRecvThread\n");
        exit(1);
    }
    this->callBack_ = _callBack;
    this->threadStopFlag_ = false;

    if(_callBack == NULL)
    {
        printError("_callBack should set when work as server!\n");
        return THREAD_CALLBACK_FAIL;
    }
    this->threadHandler_ = new std::thread(&udpTool::serverWorkThread, this);
    return ERROR_NUM::SUCCESS;
}

ERROR_NUM udpTool::endRecvThread()
{
    this->threadStopFlag_ = true;
    
    // 关闭 socket 以中断阻塞的 recvfrom
    if(this->fd_ > 0)
    {
        shutdown(this->fd_, SHUT_RDWR);
    }
    
    if(this->threadHandler_ != NULL)
    {
        if(this->threadHandler_->joinable())
        {
            this->threadHandler_->join();
        }
    }
    this->threadHandler_ = NULL;
    return ERROR_NUM::SUCCESS;
}

ERROR_NUM udpTool::destroryNet()
{
    if(this->dataBuf_ != NULL)
    {
        delete[] this->dataBuf_;
        this->dataBuf_ = NULL;
        this->bufSize_ = 0;
        this->dataSize = 0;
    }
    if(this->fd_ > 0)
    {
        close(this->fd_);
        this->fd_ = -1;
    }
    // printLog(DEBUG, "Destroyed\n");
    return ERROR_NUM::SUCCESS;
}

NETDATALEN udpTool::getData(void* buf, int len)
{
    int res = -1;
    if(this->threadStopFlag_)
    {
        printError("Receive thread not exist, must call startRecvThread() to create thread\n");
        exit(1);
    }
    this->dataAccess_.lock();
    if(this->dataSize > 0)
    {
        if(this->dataSize > len)
        {
            printError("get data: Buf too small!\n");
        }
        else 
        {
            memcpy(buf, this->dataBuf_, this->dataSize);
            res = this->dataSize;
            
            this->dataSize = 0;
        }
    }
    this->dataAccess_.unlock();
    return res;
}

NETDATALEN udpTool::sendData(void* buf, int len, struct ipPort _destIpPort)
{
    ssize_t res = -1;
    if(this->fd_ < 0)
    {
        printError("Socket not created, must call createNet() to create socket before send data\n");
        exit(1);
    }
    sockaddr_in client_addr;
    client_addr.sin_family = AF_INET;
    client_addr.sin_port = htons(_destIpPort.port);
    inet_pton(AF_INET, _destIpPort.ip.c_str(), &client_addr.sin_addr);
    res = sendto(this->fd_, buf, len, 0, (sockaddr*)&client_addr, sizeof(client_addr));
    return res;
}




/* -------------TCP----------- */
tcpTool::tcpTool(struct ipPort _sourceIpPort, MODULE_TYPE moduleId) : netWorkBase(_sourceIpPort, moduleId)
{
    this->threadStopFlag_ = true;
    this->destIpPort_ = {};
};

tcpTool::~tcpTool()
{
    this->endRecvThread();
    this->destroryNet();
}

ERROR_NUM tcpTool::createNet(int _bufSize)
{
    // tcp
    sockaddr_in address;
    this->fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if(this->fd_ < 0)
    {
        printError("Socket create failed!\n");
        return SOCK_CREATE_FAIL;
    }
    address.sin_family = AF_INET;
    address.sin_port = htons(this->sourceIpPort_.port);
    if(!this->sourceIpPort_.ip.empty())
    {
        if (inet_pton(AF_INET, this->sourceIpPort_.ip.c_str(), &address.sin_addr) <= 0) {
            printError("Ip attach failed!\n");
            return SOCK_CREATE_FAIL;
        }
    }
    else {
        /* 本机可用ip localhost */
        address.sin_addr.s_addr = INADDR_ANY;
    }
    if (bind(this->fd_, (sockaddr*)&address, sizeof(address)) < 0) {
        printError("Socket bind failed!\n");
        return SCOKET_BIND_FAIL;
    }
    this->bufSize_ = _bufSize;
    this->dataBuf_ = new char[_bufSize];
    if(this->dataBuf_ == nullptr)
    {
        printError("alloc socket data buf failed!\n");
        return SCOKET_BIND_FAIL;
    }
    return SUCCESS;
}

void tcpTool::serverWorkThread()
{
    int client_fd = -1;
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    char* socketBuf = new char[this->bufSize_];
    if(!socketBuf)
    {
        printLog(ERROR, "Thread buf alloc failed\n");
        return;
    }
    // set unblock
    int flags = fcntl(this->fd_, F_GETFL, 0);
    fcntl(this->fd_, F_SETFL, flags | O_NONBLOCK);
    // listen will do three times shakehand
    if (listen(this->fd_, 3) < 0) {
        printLog(ERROR, "Server Listen failed!\n");
        goto end;
    }
    while(!this->threadStopFlag_)
    {
        /* This while used to stop thread, when accept() */
        while(!this->threadStopFlag_)
        {
            // waiting for connect
            client_fd = accept(this->fd_, (sockaddr*)&client_addr, &addr_len);
            if (client_fd == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                } else {
                    printLog(ERROR, "accept error");
                    goto end;
                }
            } else {
                // accepted
                this->connectedFlag_ = true;
                this->connected_fd = client_fd;
                break;
            }
        }
        while(!this->threadStopFlag_)
        {
            int bytes_read = read(client_fd, socketBuf, this->bufSize_);
            if(bytes_read > 0)
            {
                this->dataAccess_.lock();
                memcpy(this->dataBuf_, socketBuf, bytes_read);
                this->dataSize = bytes_read;
                this->dataAccess_.unlock();
                if(this->callBack_)
                {
                    // sequence: no need to lock
                    this->callBack_(this, this->dataBuf_, bytes_read, client_addr);
                }
            }
            else
            {
                // disconnected
                // just this client disconnected. TODO:
                this->callBack_(this, NULL, -1, client_addr); 
                if(client_fd >= 0)
                {
                    // close connected socket
                    close(client_fd);
                    this->connectedFlag_ = false;
                    break;

                }
            }
        }
    }
end:
    printLog(DEBUG, "TCP Server Thread exit! please call endRecvThread() to waiting thread finished\n");
    this->threadStopFlag_ = true;
    this->destIpPort_ = {};
    client_fd = -1;
    this->connected_fd = -1;
    delete[] socketBuf;
}


ERROR_NUM tcpTool::startRecvThread(callBackFunc _callBack)
{
    
    if(this->fd_ < 0)
    {
        printError("Socket not created, must call createNet() to create socket before startRecvThread\n");
        exit(1);
    }
    this->callBack_ = _callBack;
    this->threadStopFlag_ = false;

    if(_callBack == NULL)
    {
        printError("_callBack must be set when work as server!\n");
        return THREAD_CALLBACK_FAIL;
    }
    this->threadHandler_ = new std::thread(&tcpTool::serverWorkThread, this);
    return ERROR_NUM::SUCCESS;
}

ERROR_NUM tcpTool::endRecvThread()
{
    this->threadStopFlag_ = true;
    if(this->threadHandler_ != NULL)
    {
        if(this->threadHandler_->joinable())
        {
            this->threadHandler_->join();
        }
    }
    this->threadHandler_ = NULL;
    return ERROR_NUM::SUCCESS;
}

ERROR_NUM tcpTool::destroryNet()
{
    if(this->dataBuf_ != NULL)
    {
        delete[] this->dataBuf_;
        this->dataBuf_ = NULL;
        this->bufSize_ = 0;
        this->dataSize = 0;
    }
    if(this->fd_ > 0)
    {
        close(this->fd_);
        this->fd_ = -1;
    }
    // print(DEBUG, "Destroyed\n");
    return ERROR_NUM::SUCCESS;
}

NETDATALEN tcpTool::sendData(void* buf, int len, struct ipPort _destIpPort)
{
    ssize_t res = -1;
    if(this->fd_ < 0)
    {
        printLog(ERROR, "Socket not created, must call createNet() to create socket before send data\n");
        exit(1);
    }

    // Server
    if(this->connectedFlag_ == false)
    {
        printLog(WARNING, "No client connect\n");
        return res;
    }
    /**
     * Now just support one connect! 
     * TODO: support muti client connect!
     */
    if(this->connected_fd >= 0)
    {
        res = send(this->connected_fd, buf, len, 0);
    }

    return res;
}

NETDATALEN tcpTool::getData(void* buf, int len)
{
    int res = -1;

    if(this->threadStopFlag_)
    {
        if(this->fd_ >= 0)
        {
            // close socket
            this->endRecvThread();
            printLog(ERROR, "Socket disconnected! Please do createNet() and startRecvThread() to restart TCP Server\n");
        }
        else 
        {
            printLog(ERROR, "Receive thread not exist, must call startRecvThread() to create thread\n");
        }
        goto out;
    }
    this->dataAccess_.lock();
    if(this->dataSize > 0)
    {
        if(this->dataSize > len)
        {
            printLog(ERROR, "get data: Buf too small!\n");
        }
        else 
        {
            memcpy(buf, this->dataBuf_, this->dataSize);
            res = this->dataSize;
            
            this->dataSize = 0;
        }
    }
    this->dataAccess_.unlock();
out:
    return res;
}