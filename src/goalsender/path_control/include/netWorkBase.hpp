#ifndef __NETWORKBASE__HPP
#define __NETWORKBASE__HPP

#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "errorNum.hpp"
#include "moduleObj.hpp"


#define NETDATALEN int

struct ipPort
{
    // char* ip;
    std::string ip;
    unsigned short port;
    bool operator!=(struct ipPort& other){
        if(this->ip != other.ip || this->port != other.port)
        {
            return true;
        }
        return false;
    }
};


enum C_S_TYPE
{
    CLIENT = 0,
    SERVER
};

class netWorkBase;

/**
 * @brief receive data handle callback function
 * @param size: received data size. size <=0 means connect off(TCP) or error
 */
typedef void (*callBackFunc)(netWorkBase* nethandler, void* buf, int size, sockaddr_in client_addr);

/**
 * @brief usage:
 * @brief 1. netWork(); set source IP and PORT
 * @brief 2. createNet(); create socket
*/
class netWorkBase{
protected:
    int fd_ = -1;

    MODULE_TYPE moduleId;
    struct ipPort sourceIpPort_;
    std::thread* threadHandler_;
    std::mutex dataAccess_;
    std::atomic<bool> threadStopFlag_;

    // Data buf: used to receive data
    char* dataBuf_;     // TODO: modify to uniptr
    int bufSize_;       // max frame size
    int dataSize;       // record len of true data, <= 0 means no data

    // 
    void *privateData = nullptr;

    // 
    std::unordered_map<int, struct ipPort*> ipMapTable;

    /**
     * @brief when receive data and callBack_ is not NULL, then call callBack_
     */ 
    callBackFunc callBack_;

public:
    netWorkBase(struct ipPort _sourceIpPort, MODULE_TYPE moduleId);
    virtual ~netWorkBase(); // 父类的析构函数必须为虚函数，否则不会调用子类的析构函数

    /** 
     * @brief this func will create socket, if UDP will bind socket and can be used to send data
     * @param _bufSize: socket receive buf size (will alloc interal), should larger then your received data
    */
    virtual ERROR_NUM createNet(int _bufSize) = 0;

    /**
     * @brief this func will create thread, if _callBack != NULL, will register _callBack
     * @param _callBack: when receive data and callBack_ is not NULL, then call callBack_
     */
    virtual ERROR_NUM startRecvThread(callBackFunc _callBack) = 0;

    /**
     * @brief this func will destroy thread, receive data will stop, send data can still work
     */
    virtual ERROR_NUM endRecvThread() = 0;

    /**
     * @brief this func will destroy socket and socket buf, then all socket func can not work
     */
    virtual ERROR_NUM destroryNet() = 0;

    /**
     * @brief 1. Client API: Get data from Server, must when startRecvThread() has been called!
     * @brief 2. Server can not use this api, please register callback to receive data from client
     * @brief 3. This API no need to be used, just for test.*********
     * @param buf: receive buf, used to receive data
     * @param len: receive buf length(size)
     */
    virtual NETDATALEN getData(void* buf, int len) = 0;

    /**
     * @brief send data to _destIpPort, this func can be used when createNet() called
     * @param buf: data will be sended
     * @param len: data length(size)
     * @param _destIpPort: destination IP and PORT
     */
    virtual NETDATALEN sendData(void* buf, int len, struct ipPort _destIpPort) = 0;

    /**
     * @brief send data to _destIpPort by OBJ Module id, this func can be used when createNet() called
     * @param buf: data will be sended
     * @param len: data length(size)
     * @param destObjIndex: destination Obj module id
     */
    virtual NETDATALEN sendDataByMap(void* buf, int len, unsigned int destObjIndex);  // 默认 type=0
    virtual NETDATALEN sendDataByMap(void* buf, int len, unsigned int destObjIndex,int frameType);

    /**
     * @brief set private data
     * @param privateData_: the private data to set
     */
    virtual ERROR_NUM setPrivateData(void *privateData_);

    /**
     * @brief get private data
     */
    virtual void* getPrivateData(void);

    /**
     * @brief register ip and port map to module id
     * @param ip_: module id ip addr
     * @param port_: module id port
     * @param destObjIndex: destination Obj module id
     */
    virtual ERROR_NUM registerDestIpMap(std::string ip_, unsigned short port_, int destObjIndex);

protected:
    virtual void serverWorkThread() = 0;
};




#endif