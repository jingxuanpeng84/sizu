#ifndef OCTI_SPEAKER__HPP
#define OCTI_SPEAKER__HPP
#include <iostream>
#include <queue>
#include <mutex>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>

namespace OCTI_SPEAKER
{
    class speaker
    {
    public:
        speaker(const char *serverIp_, unsigned short serverPort_, const char *myIp_, unsigned short myPort_);
        ~speaker();
        bool IsmessageEmpty();
        bool addMessage(std::string message_);
        std::string getMessage();
        void deleteMessage();

        int socket_fd;
    private:
        
        unsigned int max_capcity = 2;
        std::mutex message_queue_mutex;
        std::queue<std::string> *message_queue;
    };

    void speakerThread(OCTI_SPEAKER::speaker* speaker_);
}

#endif