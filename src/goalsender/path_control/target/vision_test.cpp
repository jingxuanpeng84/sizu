#include "path_control/include/vision_device.hpp"
#include "path_control/include/octi_yaml.hpp"
#include "path_control/include/octi_robot.hpp"

int main()
{
    std::cout << "Loading config.yaml\n";
    octi_yaml yaml("/home/dog/Desktop/OctiRobotVersion2/path_control/target/config/config.yaml");
    std::string visionIp = "nan";
    std::string robotVisionIp = "nan";
    uint32_t visionPort = 0;
    uint32_t visionLoad = 0;
    uint32_t robotVisionPort = 0;

    std::string myIp = "nan";
    uint32_t myPort = 0;

    std::string robotNetInterface = "nan";
    uint32_t robotActionDurationTime = 0;

    // load robotNetInterface
    if (yaml.IsDefine("robotNetInterface") == false)
    {
        perror("target/config/config.yaml: Please define robotNetInterface !");
        exit(1);
    }
    robotNetInterface = yaml.getString("robotNetInterface");
    std::cout << "robotNetInterface = " << robotNetInterface << std::endl;

    // load robotActionDurationTime
    if (yaml.IsDefine("robotActionDurationTime") == false)
    {
        perror("target/config/config.yaml: Please define robotActionDurationTime !");
        exit(1);
    }
    robotActionDurationTime = yaml.getUint("robotActionDurationTime");
    std::cout << "robotActionDurationTime = " << robotActionDurationTime << std::endl;

    // load vision
    if (yaml.IsDefine("visionLoad") == true)
    {
        if (yaml.getUint("visionLoad") == 1)
        {
            if (yaml.IsDefine("visionIp") == false || yaml.IsDefine("visionPort") == false || yaml.IsDefine("robotVisionPort") == false || yaml.IsDefine("robotVisionIp") == false)
            {
                perror("target/config/config.yaml: Please define visionIp / visionPort / robotVisionPort / robotVisionIp, when need load vision !");
                exit(1);
            }
            visionLoad = 1;
            visionIp = yaml.getString("visionIp");
            visionPort = yaml.getUint("visionPort");
            robotVisionIp = yaml.getString("robotVisionIp");
            robotVisionPort = yaml.getUint("robotVisionPort");
            std::cout << "visionIp = " << visionIp << " visionPort = " << visionPort << " robotVisionPort = " << robotVisionPort << " robotVisionIp = " << robotVisionIp << std::endl;
        }
    }
    else
    {
        perror("Vision must been loaded");
        exit(0);
    }

    // load my ip and my port
    if (yaml.IsDefine("myIp") == false || yaml.IsDefine("myPort") == false)
    {
        perror("target/config/config.yaml: Please define myIp / myPort !");
        exit(1);
    }
    myIp = yaml.getString("myIp");
    myPort = yaml.getUint("myPort");
    std::cout << "myIp = " << myIp << " myPort = " << myPort << std::endl;

    deviceBase *myVision = new visionDevice(visionIp.c_str(), visionPort, robotVisionIp.c_str(), robotVisionPort);
    if (myVision->startWork() == false)
    {
        perror("Vision start error");
        exit(0);
    }

    // create motion_control
    octiRobot myrobot = octiRobot(robotNetInterface.c_str(), NULL, robotActionDurationTime);
    // communicate with vision
    VISION::dataStruct visionData;
    VISION::dataStruct visionRecData;
    visionData.instruction = VISION::instructionTypeEnum::REQUEST;
    unsigned int visionSize = sizeof(visionData);
    myVision->sendDeviceData(&visionData, visionSize);
    /*
    test
    */
    // myrobot.octiBalanceStand();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    myrobot.octiEuler(0, -0.35, 0);
    while (true)
    {
        if (myVision->readDeviceData(&visionRecData, visionSize) == false)
        {
            std::cout << "Laddar may disconnect! Please check!\n";
            myrobot.octiEuler(0, 0, 0);
            break; // finish vision
        }
        else if (visionRecData.instruction == VISION::instructionTypeEnum::ACK)
        {
            myrobot.octiEuler(0, -0.35, 0);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            visionData.instruction = VISION::instructionTypeEnum::ALIVE;
            myVision->sendDeviceData(&visionData, visionSize);
        }
        else
        {
            myrobot.octiEuler(0, 0, 0);
            break; // finish vision
        }
    }
    myVision->endWork();
    myVision->watingDeviceEnding();
    delete myVision;
    return 0;
}