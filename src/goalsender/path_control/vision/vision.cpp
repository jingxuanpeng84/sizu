#include "vision.hpp"

OCTI_VISION::octiVision::octiVision(const char *dest_ip_, unsigned short dest_port_, const char *my_ip_, unsigned short my_port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::cout << "sock error\n";
        exit(1);
    }
    this->mysocketfd = fd;
    this->destIp = dest_ip_;
    this->destPort = dest_port_;
    this->myIp = my_ip_;
    this->myPort = my_port;
    sockaddr_in my;
    memset(&my, 0, sizeof(my));
    my.sin_family = AF_INET;
    my.sin_port = htons(my_port);
    my.sin_addr.s_addr = inet_addr(my_ip_);
    int ret = bind(fd, (sockaddr *)&my, sizeof(my));
    if (ret < 0)
    {
        exit(2);
    }
}
OCTI_VISION::octiVision::~octiVision()
{
    // this->vision_th->join();
    close(this->mysocketfd);
}

void OCTI_VISION::vision_thread(OCTI_VISION::octiVision* vison)
{
    // 先发数据请求
    sockaddr_in server;
    sockaddr_in send_c;
    socklen_t size = sizeof(send_c);
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(vison->destIp);
    server.sin_port = htons(vison->destPort);
    visiondatastruct send_data;
    visiondatastruct receive_data;
    send_data.command = 1;
    ssize_t num = 0;
    int flags = fcntl(vison->mysocketfd, F_GETFL, 0);
    fcntl(vison->mysocketfd, F_SETFL, flags | O_NONBLOCK);
    do
    {
        sendto(vison->mysocketfd, &send_data, sizeof(send_data), 0, (sockaddr *)&server, sizeof(server));
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(vison->mysocketfd, &readfds);
        //
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 27000;
        //

        int result = select(vison->mysocketfd + 1, &readfds, NULL, NULL, &timeout);
        if (result < 0)
        {
            perror("select failed");
            close(vison->mysocketfd);
            exit(EXIT_FAILURE);
        }
        else if (result == 0)
        {
            continue;
        }

        // usleep(1000);
        num = recvfrom(vison->mysocketfd, &receive_data, sizeof(receive_data), 0, (sockaddr *)&send_c, &size);
        if (num > 0 && receive_data.command == 2)
        {
            
            vison->vision_data_mutex.lock();
            vison->vision_data = receive_data;
            vison->data_update_flag = OCTI_VISION::DATA_UPDATE::DATA;
            vison->vision_data_mutex.unlock();
            std::this_thread::sleep_for(std::chrono::microseconds(100000)); //100000 us -> 100ms
        }
    } while (1);
}

OCTI_VISION::vision_return_data OCTI_VISION::octiVision::getVisondata()
{
    vision_return_data data;
    this->vision_data_mutex.lock();
    
    if (this->data_update_flag == OCTI_VISION::DATA_UPDATE::DATA)
    {
        // std::cout << "data\n";
        // std::cout << "flag = " << this->data_update_flag << std::endl;
        data.flag = OCTI_VISION::DATA_UPDATE::DATA;
        data.distance = this->vision_data.distance;
        data.flag_obs = this->vision_data.flag_obs;
        data.left_width = this->vision_data.left_width;
        data.right_width = this->vision_data.right_width;
        data.width = this->vision_data.width;
        this->data_update_flag = OCTI_VISION::DATA_UPDATE::NODATA;
        this->vision_data_mutex.unlock();
    }
    else
    {
        this->vision_data_mutex.unlock();
        data.flag = OCTI_VISION::DATA_UPDATE::NODATA;
    }
    return data;
}
