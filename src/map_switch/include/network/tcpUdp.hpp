#ifndef __TCP_UDP__HPP
#define __TCP_UDP__HPP

#include "netWorkBase.hpp"


class udpTool : public netWorkBase {
protected:
    void serverWorkThread() override;
public:
    udpTool(struct ipPort _sourceIpPort, MODULE_TYPE moduleId);
    ~udpTool();
    ERROR_NUM createNet(int _bufSize) override;
    ERROR_NUM startRecvThread(callBackFunc _callBack) override;
    ERROR_NUM endRecvThread() override;
    ERROR_NUM destroryNet() override;

    NETDATALEN getData(void* buf, int len) override;
    NETDATALEN sendData(void* buf, int len, struct ipPort _destIpPort) override;
};


/**
 * @brief depended connect
 * @brief Version 1.0 < 1. default to bind, whatever client or server. 2. one to one >
 * @brief TODO: Version 2.0 < server use select/epoll. one to mutiple at same time >
 */
class tcpTool : public netWorkBase {
private:
    /*  */
    bool connectedFlag_ = false;
    /* Just for client */
    struct ipPort destIpPort_;

    /* Just for server. TODO: improve to more connected fd */
    int connected_fd = -1;
protected:
    void serverWorkThread() override;

public:
    tcpTool(struct ipPort _sourceIpPort, MODULE_TYPE moduleId);
    ~tcpTool();
    ERROR_NUM createNet(int _bufSize) override;
    ERROR_NUM startRecvThread(callBackFunc _callBack) override;
    ERROR_NUM endRecvThread() override;
    ERROR_NUM destroryNet() override;

    NETDATALEN getData(void* buf, int len) override;
    NETDATALEN sendData(void* buf, int len, struct ipPort _destIpPort) override;
};

#endif