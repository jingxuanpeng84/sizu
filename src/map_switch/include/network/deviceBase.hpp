#ifndef __DEVICEBASE__HPP
#define __DEVICEBASE__HPP

#include <stdlib.h>
#include <string.h>
#include <string>
#include <queue>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>
#include"moduleObj.hpp"
#include "netWorkBase.hpp"

enum DEVICE_TYPE
{
    HTTP_SERVER = 0,
    VISION,
    DEVICE_NUM,
};

enum DEVICE_STATUS
{
    RUNNING,
    STOP,
    UNREACH,
};

// 心跳状态枚举
// enum HeartbeatStatus
// {
//     NORMAL = 0,   // 正常心跳
//     ABNORMAL = 1  // 异常心跳
// };

enum CHECK_STATUS{
    CHECK_ERROR = -1,
    CHECK_WAITING =0,
    CHECK_NORMAL =1
};

class device;

typedef void (*deviceCallback)(class device* devHandler);
typedef std::pair<std::string,int> (*resetDevice)(class device* devHandler);
typedef std::pair<std::string,int> (*checkCallback)(class device* devHandler);

struct DeviceCallbackTable{
    deviceCallback dataCallback; //数据处理回调
    resetDevice resetCallback; //重启回调
    checkCallback checkCallback_; //状态检查回调

    DeviceCallbackTable(
        deviceCallback data = nullptr,
        resetDevice reset = nullptr,
        checkCallback check = nullptr
    ) : dataCallback(data),
        resetCallback(reset),
        checkCallback_(check) {}
};


class device {
private:
    std::string deviceName;//对端模块的名字
    std::string moduleName;//本模块的名字
    std::mutex mtx;
    std::condition_variable cond;
    std::queue<std::pair<char*, int>> dataQueue;
    std::mutex queueAccessMutex;
    std::thread *deviceThreadHandler;
    std::atomic<DEVICE_STATUS> devStatus;


    // device check status
    // -1: error, need immediate reset
    //  0: waiting for watchdog feed
    //  1: watchdog fed, normal
    std::atomic<int> checkStatus;

    std::atomic<bool> threadStop;

    class netWorkBase* communicator;
    DeviceCallbackTable callbackTable;

    unsigned int heartbeatDestModule; 
    // 心跳相关
    std::atomic<bool> heartbeatRunning;
    std::thread *heartbeatThreadHandler;
    //std::atomic<HeartbeatStatus> heartbeatStatus;
    //std::mutex heartbeatMtx;

    // resetDevice resetDevHandler;
    // checkCallback checkDevHandler;

private:
    // 心跳线程函数
    void heartbeatLoop();
    // 异常状态上报前端
    //void reportToFrontend();
    void sendHeartbeat(const std::string& msg ,int status,unsigned int destModule = HTTPSERVER_OBJ);

public:
    device(std::string deviceName_, class netWorkBase* communicator_,DeviceCallbackTable callbackTable_ ,unsigned int heartbeatDest_ = HTTPSERVER_OBJ);

    ~device();

    std::string getDeviceName(void);

    std::pair<std::string,int> ResetModule(void)
    {
        if(callbackTable.resetCallback == nullptr)
        {
            return std::make_pair("NO_RESET_HANDLER", -1);
        }
        return callbackTable.resetCallback(this);
    }
    
    int pushData(char *buf_, int size);
    
    std::pair<char*, int> frontData(void);

    void popData(void);

    size_t dataSize(void);

    void requestRestart(const std::string& reason,unsigned int destModule = HTTPSERVER_OBJ);




    /**
     * @brief send data to _destIpPort by OBJ Module id, this func can be used when createNet() called
     *          just need to send data part, no need send frame head, this api interal will set frame head.
     * @param buf: data will be sended
     * @param len: data length(size)
     * @param destObjIndex: destination Obj module id
     */
    int sendDataTo(void* buf, int len, unsigned int destObjIndex);
   
    void devThread(void);
  

    // 启动心跳线程
    void startHeartbeat();
 
    // 停止心跳线程
    void stopHeartbeat();
    //
    //正常喂狗
    void feedWatchdog();

    //主动报错
    void reportError();

    void setCheckStatus(int status);

};

#endif