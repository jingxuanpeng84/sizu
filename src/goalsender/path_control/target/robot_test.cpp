#include "path_control/include/octi_robot.hpp"
#include "path_control/include/octi_yaml.hpp"

int main()
{
    std::cout << "Loading config.yaml\n";
    octi_yaml yaml("/home/orangepi/Desktop/work_station/OctiRobotVersion2/path_control/target/config/config.yaml");
    std::string robotNetInterface = "nan";
    uint32_t robotActionDurationTime = 0;

    uint32_t motionInterfaceLoad = 0;
    std::string motionInterfaceIp = "nan";
    uint32_t motionInterfacePort = 0;

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

    if (yaml.IsDefine("motionInterfaceLoad") == true)
    {
        if (yaml.getUint("motionInterfaceLoad") == 1)
        {
            if (yaml.IsDefine("motionInterfaceIp") == false || yaml.IsDefine("motionInterfacePort") == false)
            {
                perror("target/config/config.yaml: Please define motionInterfaceIp / motionInterfacePort, when need load speaker !");
                exit(1);
            }
            motionInterfaceLoad = 1;
            motionInterfaceIp = yaml.getString("motionInterfaceIp");
            motionInterfacePort = yaml.getUint("motionInterfacePort");
            std::cout << "motionInterfaceIp = " << motionInterfaceIp << " motionInterfacePort = " << motionInterfacePort << std::endl;
        }
    }

    octiRobot myrobot = octiRobot(robotNetInterface.c_str(), NULL, robotActionDurationTime);
    // myrobot.startMotionInterface(motionInterfaceIp.c_str(), motionInterfacePort);
    std::cout << "robot battery = " << int(myrobot.getBatteryVolum()) << std::endl;

    double data;
    myrobot.getRobotState( "speedLevel", data);
    std::cout << "robot speedLevel = " << data << std::endl;
    myrobot.getRobotState("gait", data);
    std::cout << "robot gait = " << data << std::endl;
    if (myrobot.isBatteryCharging())
    {
        std::cout << "robot charging !\n";
    }
    else
    {
        std::cout << "robot not charging !\n";
    }
    // myrobot.octiStandUp();
    // myrobot.octiMove(0.05, 0, 0);
    // sleep(5);

    // myrobot.octiBalanceStand();
    // myrobot.octiEuler(0, -0.35, 0);

    
                        for (int i = 0; i < 3; i++) {
                        myrobot.octiMove(0, 0, 0.82);
                        sleep(1);  // 休眠1秒
                        }
    sleep(3);
    // myrobot.octiStandDown();



    // while(1)
    // {
    //     std::this_thread::sleep_for(std::chrono::microseconds(200));
    // }
    // myrobot.octiStopLock();
    // myrobot.octiMove(0.4, 0, 0);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // int i = 30;
    // while (i >= 0)
    // {
    //     myrobot.octiEuler(0, -0.35, 0);
    //     std::this_thread::sleep_for(std::chrono::microseconds(200000));
    //     i--;
    // }
    // std::this_thread::sleep_for(std::chrono::seconds(5));
    // myrobot.octiEuler(0, 0, 0);
   
    return 0;
}