#include "./speaker.hpp"

int main()
{
    OCTI_SPEAKER::speaker myspeaker("192.168.124.123", 8001, "192.168.124.110", 6004);

    std::thread spTask(OCTI_SPEAKER::speakerThread, &myspeaker);

    std::string command;
    while(1)
    {   
        std::cin >> command;
        if(command == "1")
        {
            //stop
            close(myspeaker.socket_fd);
            exit(1);
        }
        myspeaker.addMessage("[b0]前方障碍物太宽无法避开，请移走障碍物[d]");
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
        std::cout << "next\n";
        myspeaker.addMessage("[b0]前方物体太近，紧急停止[d]");
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
    
    spTask.join();
}