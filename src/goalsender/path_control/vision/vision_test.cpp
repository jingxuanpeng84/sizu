#include "vision.hpp"

void task2(OCTI_VISION::octiVision* vision)
{
    printf("t2 %p\n", vision);
    while (1)
    {
        OCTI_VISION::vision_return_data data = vision->getVisondata();
        if (data.flag == OCTI_VISION::DATA_UPDATE::DATA)
        {
            if (data.flag_obs == 1)
            {
                std::cout << "Exit obs : dist = " << data.distance << " len = " << data.distance << std::endl;
            }
            else
            {
                std::cout << "data rec but no obs\n";
            }
        }
        else
        {
            // std::cout << "no data\n";
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
}

int main()
{
    OCTI_VISION::octiVision vision("192.168.123.123", 6000, "192.168.123.110", 6001);
    std::thread th2(task2, &vision);
    std::thread vision_thread_t(OCTI_VISION::vision_thread, &vision);
    th2.join();
    vision_thread_t.join();
}