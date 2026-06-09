#ifndef OCTI_VISION__HPP
#define OCTI_VISION__HPP
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <thread>
#include <mutex>

namespace OCTI_VISION
{
    struct visiondatastruct
    {
        int command;
        int flag_obs;
        float width;
        float distance;
        float right_width;
        float left_width;
    };

    struct vision_return_data
    {
        int flag;
        int flag_obs;
        float width;
        float distance;
        float right_width;
        float left_width;
    };

    enum VISION_STATE
    {
        VISION_OK,
        VISION_ERROR,
    };

    enum DATA_UPDATE
    {
        DATA,
        NODATA,
    };
    
    class octiVision
    {
    public:
        octiVision(const char *dest_ip_, unsigned short dest_port_, const char *my_ip_, unsigned short my_port);
        ~octiVision();

        vision_return_data getVisondata();

        // void create_Vision_Thread();

        // private:
        int mysocketfd;
        const char *destIp;
        unsigned short destPort;
        const char *myIp;
        unsigned short myPort;
        int data_update_flag = DATA_UPDATE::NODATA;
        visiondatastruct vision_data;
        std::mutex vision_data_mutex;
    };
    void vision_thread(OCTI_VISION::octiVision *vison);
};

#endif