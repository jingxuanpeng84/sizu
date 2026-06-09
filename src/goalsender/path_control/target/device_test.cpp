#include "path_control/include/laddar_device.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

int main(int argc, char **argv)
{
    // Default test parameters. You can override by command line:
    // deviceTest <laddar_ip> <laddar_port> <my_ip> <my_port> [interval_ms]
    std::string laddarIp = "192.168.31.126";
    unsigned short laddarPort = 6003;
    std::string myIp = "192.168.31.70";
    unsigned short myPort = 6002;
    int intervalMs = 200;

    if (argc >= 5)
    {
        laddarIp = argv[1];
        laddarPort = static_cast<unsigned short>(std::stoi(argv[2]));
        myIp = argv[3];
        myPort = static_cast<unsigned short>(std::stoi(argv[4]));
    }
    if (argc >= 6)
    {
        intervalMs = std::max(10, std::stoi(argv[5]));
    }

    std::cout << "[DeviceTest] laddar=" << laddarIp << ":" << laddarPort
              << ", local=" << myIp << ":" << myPort
              << ", read_interval_ms=" << intervalMs << std::endl;

    std::unique_ptr<deviceBase> laddar(new laddarDevice(
        laddarIp.c_str(),
        laddarPort,
        myIp.c_str(),
        myPort));

    if (!laddar->startWork())
    {
        std::cerr << "[DeviceTest] startWork failed, check IP/port and network." << std::endl;
        return 1;
    }

    LADDAR::dataStruct laddarData {};
    unsigned int dataSize = 0;
    unsigned long okCount = 0;
    unsigned long failCount = 0;
    auto startTs = std::chrono::steady_clock::now();
    auto lastStatTs = startTs;

    while (true)
    {
        if (!laddar->readDeviceData(&laddarData, dataSize))
        {
            failCount++;
            std::cout << "[DeviceTest] read timeout/disconnect (fail_count=" << failCount << ")" << std::endl;

            // Too many continuous failures means link is not healthy.
            if (failCount >= 10)
            {
                std::cerr << "[DeviceTest] continuous failures reached limit, stop test." << std::endl;
                break;
            }
        }
        else
        {
            okCount++;
            failCount = 0;

            std::cout << std::fixed << std::setprecision(3)
                      << "[DeviceTest] seq=" << okCount
                      << " pos=(" << laddarData.x << ", " << laddarData.y << ")"
                      << " yaw=" << laddarData.yaw
                      << " dist[0]=" << laddarData.dist[0]
                      << " dist[90]=" << laddarData.dist[90]
                      << " dist[180]=" << laddarData.dist[180]
                      << std::endl;
        }

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastStatTs).count() >= 5)
        {
            double sec = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTs).count() / 1000.0;
            double hz = sec > 0.0 ? static_cast<double>(okCount) / sec : 0.0;
            std::cout << std::fixed << std::setprecision(2)
                      << "[DeviceTest] stats: ok=" << okCount
                      << ", fail=" << failCount
                      << ", avg_hz=" << hz
                      << std::endl;
            lastStatTs = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    laddar->endWork();
    laddar->watingDeviceEnding();
    return 0;
}