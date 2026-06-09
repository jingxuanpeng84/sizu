#include "./speaker.hpp"

OCTI_SPEAKER::speaker::speaker(const char* serverIp_, unsigned short serverPort_, const char* myIp_, unsigned short myPort_)
{
    this->message_queue = new std::queue<std::string>;
    this->socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(this->socket_fd < 0)
    {
        std::cout << "socket error\n";
        exit(1);
    }
    sockaddr_in my;
    memset(&my, 0, sizeof(my));
    my.sin_family = AF_INET;
    my.sin_port = htons(myPort_);
    my.sin_addr.s_addr = inet_addr(myIp_);
    if(bind(this->socket_fd, (sockaddr *)&my, sizeof(my)) < 0)
    {
        perror("bind fail");
        exit(2);
    }
    sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port = htons(serverPort_);
    server.sin_addr.s_addr = inet_addr(serverIp_);
    if(connect(this->socket_fd, (struct sockaddr*)&server, sizeof(server)) < 0)
    {
        perror("connect fail");
        exit(2);
    }
}

OCTI_SPEAKER::speaker::~speaker()
{
    if(this->socket_fd > 0)
    {
        close(this->socket_fd);
    }
    if(this->message_queue != NULL)
    {
        delete this->message_queue;
    }
}

bool OCTI_SPEAKER::speaker::IsmessageEmpty()
{
    return this->message_queue->empty();
}

bool OCTI_SPEAKER::speaker::addMessage(std::string message_)
{
    this->message_queue_mutex.lock();
    if(this->message_queue->size() >= this->max_capcity)
    {
        this->message_queue_mutex.unlock();
        return false;
    }
    else
    {
        this->message_queue->push(message_);
        // std::cout << "size = " << this->message_queue->size() << std::endl;
        this->message_queue_mutex.unlock();
        return true;
    }
}

std::string OCTI_SPEAKER::speaker::getMessage()
{
    std::string data; 
    this->message_queue_mutex.lock();
    if(this->message_queue->empty())
    {
        this->message_queue_mutex.unlock();
        return data;
    }
    else {
        data = this->message_queue->front();
        // this->message_queue->pop();
        this->message_queue_mutex.unlock();
        return data;
    }
}

void OCTI_SPEAKER::speaker::deleteMessage()
{
    // std::cout << "delete\n";
    this->message_queue_mutex.lock();
    // std::cout << " size = " << this->message_queue->size() << std::endl;
    if(this->message_queue->size() > 0)
    {
        this->message_queue->pop();
    }
    this->message_queue_mutex.unlock();
}

void OCTI_SPEAKER::speakerThread(OCTI_SPEAKER::speaker* speaker_)
{
    std::string message_data;
    char recbuf[64];
    unsigned int rec_flag = 1;
    while(1)
    {
        if(speaker_->IsmessageEmpty())
        {
            continue;
        }
        message_data = speaker_->getMessage();
        if(message_data.size() > 0)
        {
            if( rec_flag == 1)
            {
                //be able to send
                write(speaker_->socket_fd, message_data.c_str(), message_data.size());
                rec_flag = 0;
                int num = read(speaker_->socket_fd, recbuf, 64);
                if(num < 0)
                {
                    continue;
                }
                else
                {
                    // std::cout << "rec res\n";
                    if(strcmp("ok", recbuf) == 0)
                    {
                        // std::cout << "rec ok\n";
                        speaker_->deleteMessage();
                        memset(recbuf, 0, 10);
                        rec_flag = 1;
                    }
                }
            }
            else {
                //can not send
                continue;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
}