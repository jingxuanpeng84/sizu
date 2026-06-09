#include "/home/orangepi/slam_ws/src/goalsender/include/network/deviceBase.hpp"
#include"/home/orangepi/slam_ws/src/goalsender/include/network/frame.hpp"

void device::sendHeartbeat(const std::string& msg ,int status,unsigned int destModule)
{   //计算info[]的大小
    int infolen = msg.length();
    int heartbeatFrameSize = sizeof(struct heartdancefra) + infolen;
    //分配内存
    struct heartdancefra* hbFrame =(struct heartdancefra*)malloc(heartbeatFrameSize);
    //填充心跳帧字段
    hbFrame->type = HEARTBEAT_STATUS;//改为宏定义
    hbFrame->status =status;
    hbFrame->len = infolen;
    memcpy(hbFrame->info,msg.c_str(),infolen);

    if(communicator){
        communicator->sendDataByMap((void*)hbFrame,heartbeatFrameSize,destModule,FRAME_TYPE_HEARTBEAT);
    }

    free(hbFrame);
}

//本module的心跳线程
void device::heartbeatLoop()
{
    
    while (heartbeatRunning)
    {

        std::string heartbeatMsg;
        int statusCode = 0;
        //调用checkcallback检查本模块的状态
        if (callbackTable.checkCallback_){
            std::pair<std::string,int> resInfo = this -> callbackTable.checkCallback_(this);
            heartbeatMsg = resInfo.first;
            statusCode = resInfo.second;
        }else{
            heartbeatMsg = "NO_CHECK_HANDLER";
            statusCode = -1;
        }


        // 主动发送心跳包到服务器
        sendHeartbeat(heartbeatMsg,statusCode,heartbeatDestModule);
        // if (communicator)
        // {
        //     // 发送心跳包，destObjIndex根据实际情况设置
        //     communicator->sendDataByMap((void *)heartbeatMsg.c_str(), heartbeatMsg.length(), 0);
        // }

        // 1Hz频率：每秒发送一次心跳
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// 异常状态上报前端
// void device::reportToFrontend()
// {
//     // 这里可以根据实际需求实现上报逻辑
//     // 例如：通过communicator发送异常消息
//     const char* errorMsg = "HEARTBEAT_ABNORMAL";
//     if(communicator)
//     {
//         communicator->sendDataByMap((void*)errorMsg, strlen(errorMsg), 0);
//     }
// }

device::device(std::string deviceName_, class netWorkBase *communicator_, DeviceCallbackTable callbackTable_,unsigned int heartbeatDest_) //
{
    deviceName = deviceName_;
    communicator = communicator_;
    threadStop = false;
    callbackTable = callbackTable_;
    heartbeatDestModule = heartbeatDest_;
    checkStatus = CHECK_WAITING; // 等待喂狗
    deviceThreadHandler = new std::thread(&device::devThread, this);
    devStatus = DEVICE_STATUS::RUNNING;
    heartbeatRunning = false;
    heartbeatThreadHandler = nullptr;
    // heartbeatStatus = NORMAL;
}

device::~device()
{
    threadStop = true;
    cond.notify_all();  // 通知等待的线程退出
    if (deviceThreadHandler->joinable())
    {
        deviceThreadHandler->join();
    }
    deviceThreadHandler = nullptr;

    // 停止心跳线程
    stopHeartbeat();

    communicator = nullptr;
    devStatus = DEVICE_STATUS::STOP;
}

std::string device::getDeviceName(void)
{
    return this->deviceName;
}
int device::pushData(char *buf_, int size)
{
    void *data = malloc(sizeof(char) * size);
    memcpy(data, buf_, size);
    queueAccessMutex.lock();
    this->dataQueue.push(std::make_pair((char *)data, size));
    queueAccessMutex.unlock();
    cond.notify_all();
    return 0;
}

std::pair<char *, int> device::frontData(void)
{
    std::pair<char *, int> res;
    queueAccessMutex.lock();
    res = this->dataQueue.front();
    queueAccessMutex.unlock();
    return res;
}

void device::popData(void)
{
    std::pair<char *, int> data;
    size_t queueSize;
    queueAccessMutex.lock();
    queueSize = this->dataQueue.size();
    if (queueSize < 0)
    {
        queueAccessMutex.unlock();
        return;
    }
    data = this->dataQueue.front();
    this->dataQueue.pop();
    queueAccessMutex.unlock();
    if (data.first)
    {
        free(data.first);
    }
    return;
}

size_t device::dataSize(void)
{
    size_t size;
    queueAccessMutex.lock();
    size = this->dataQueue.size();
    queueAccessMutex.unlock();
    return size;
}

/**
 * @brief send data to _destIpPort by OBJ Module id, this func can be used when createNet() called
 *          just need to send data part, no need send frame head, this api interal will set frame head.
 * @param buf: data will be sended
 * @param len: data length(size)
 * @param destObjIndex: destination Obj module id
 */

int device::sendDataTo(void *buf, int len, unsigned int destObjIndex)
{
    if (buf)
        return this->communicator->sendDataByMap(buf, len, destObjIndex);
    return 0;
}

void device::devThread(void)
{
    while (!threadStop)
    {
        if (this->dataSize() > 0)
        {
            if (callbackTable.dataCallback)
            {
                callbackTable.dataCallback(this);
            }
        }
        else
        {
            std::unique_lock<std::mutex> lock(mtx);
            // 使用带超时的等待，每100ms检查一次threadStop
            cond.wait_for(lock, std::chrono::milliseconds(100));
        }
    }
}

// 启动心跳线程
void device::startHeartbeat()
{
    if (!heartbeatRunning)
    {
        heartbeatRunning = true;
        heartbeatThreadHandler = new std::thread(&device::heartbeatLoop, this);
    }
}

// 停止本module的心跳线程
void device::stopHeartbeat()
{
    if (heartbeatRunning)
    {
        heartbeatRunning = false;
        if (heartbeatThreadHandler && heartbeatThreadHandler->joinable())
        {
            heartbeatThreadHandler->join();
        }
        delete heartbeatThreadHandler;
        heartbeatThreadHandler = nullptr;
    }
}

void device::feedWatchdog()
{
    this->checkStatus = CHECK_NORMAL;
}

void device::reportError()
{
    this->checkStatus = CHECK_ERROR;
}

void device::setCheckStatus(int status)
{
    this->checkStatus = status;
}

void device::requestRestart(const std::string& reason,unsigned int destModule)
{
    int infolen = reason.length();
    int heartbeatFrameSize = sizeof(struct heartdancefra)  + infolen;
    struct heartdancefra* hbFrame = (struct heartdancefra*)malloc(heartbeatFrameSize);

    hbFrame->type = HEARTBEAT_REQUEST_RESTART;
    hbFrame->status = -1;//错误状态
    hbFrame->len = infolen;
    memcpy(hbFrame->info,reason.c_str(),infolen);

    if(communicator){
        communicator->sendDataByMap((void*)hbFrame,heartbeatFrameSize,destModule,FRAME_TYPE_HEARTBEAT);
    }

    free(hbFrame);
}