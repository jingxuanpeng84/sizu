#include "path_control/include/octi_charge.hpp"
#include "path_control/include/video_device.hpp"
#include "path_control/include/octi_robot.hpp"
#include "path_control/include/octi_yaml.hpp"

int main()
{
    std::cout << "Loading config.yaml\n";
    octi_yaml yaml("/home/orangepi/Desktop/work_station/OctiRobotVersion2/path_control/target/config/config.yaml");
    std::string robotNetInterface = "nan";
    uint32_t robotActionDurationTime = 0;

    std::string videoDeviceAddr = "nan";
    uint32_t videoDeviceLoad = 0;

    uint32_t autoChargeLoad = 0;
    uint32_t lowCharge = 0;
    uint32_t highCharge = 0;

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

    // load videoDevice
    if (yaml.IsDefine("videoDeviceLoad") == true)
    {
        if (yaml.getUint("videoDeviceLoad") == 1)
        {
            if (yaml.IsDefine("videoDeviceAddr") == false)
            {
                perror("target/config/config.yaml: Please define videoDeviceAddr, when need load videoDevice !");
                exit(1);
            }
            videoDeviceAddr = yaml.getString("videoDeviceAddr");
            videoDeviceLoad = 1;
            std::cout << "videoDeviceAddr = " << videoDeviceAddr << std::endl;
        }
    }

    videoDeviceAddr = yaml.getString("videoDeviceAddr");
    std::cout << "videoDeviceAddr = " << videoDeviceAddr << std::endl;

    // load autoCharge -> depends on video
    if (yaml.IsDefine("autoChargeLoad") == true)
    {
        if (yaml.getUint("autoChargeLoad") == 1)
        {
            if (videoDeviceLoad != 1)
            {
                perror("target/config/config.yaml: Please config video if need autocharge !");
                exit(1);
            }
            if (yaml.IsDefine("lowCharge") == false || yaml.IsDefine("highCharge") == false)
            {
                perror("target/config/config.yaml: Please define lowCharge / highCharge, when need load autoCharge !");
                exit(1);
            }
            lowCharge = yaml.getUint("lowCharge");
            highCharge = yaml.getUint("highCharge");
            autoChargeLoad = 1;
            std::cout << "autoCharge lowCharge = " << lowCharge << "%" << std::endl;
            std::cout << "autoCharge highCharge = " << highCharge << "%" << std::endl;
        }
    }

    octiRobot myrobot = octiRobot(robotNetInterface.c_str(), NULL, robotActionDurationTime);
    videoDevice myvideo = videoDevice(videoDeviceAddr.c_str());
    octiCharge mycharge = octiCharge(0.078);
    if (myvideo.startWork())
    {
        std::cout << "open success\n";
        // myvideo.openWindowShow();
    }
    cv::Mat *frame = new cv::Mat();
    unsigned int dataSize;
    // myrobot.octiStandUp();
    myrobot.setMoveSpeedLevel(OCTIROBOT::ROBOTSPEEDLEVEL::LOW);
    bool chargeFlag = false;
    int try_times = 0;
    while (!chargeFlag)
    {
        if (myvideo.readDeviceData(frame, dataSize))
        {
            // std::cout << "read success\n";
            CHARGE::aprilTagDataStruct chargeData;
            try_times = 0;
            do{
                mycharge.getCvMatData(*frame);
                chargeData = mycharge.detectAprilTag();
                if(chargeData.tag_id >= 0)
                {
                    break;
                }
                else
                {
                    try_times++;
                    if(try_times >= 6)
                    {
                        break;
                    }
                }
                myvideo.readDeviceData(frame, dataSize);
            }while(1);
            CHARGE::moveInstructionDataStruct chargeMoveData;
            CHARGE::INSTRUCTION_TYPE insType = mycharge.chargeAction(chargeData, chargeMoveData);
            switch (insType)
            {
            case CHARGE::INSTRUCTION_TYPE::MOVE:
                // std::cout << "out a = " << chargeMoveData.wVelocity << std::endl;
                myrobot.octiMove(chargeMoveData.xVelocity, chargeMoveData.yVelocity, chargeMoveData.wVelocity);
                break;
            case CHARGE::INSTRUCTION_TYPE::STANDDOWN:
                // myrobot.octiMove(0, 0, -0.3);
                // sleep(1);
                myrobot.octiStandUp();
                sleep(1.5);
                myrobot.octiStandDown();
                break;
            case CHARGE::INSTRUCTION_TYPE::FINISH:
                if (myrobot.isBatteryCharging())
                {
                    std::cout << "robot charging !\n";
                    chargeFlag = true;
                }
                else
                {
                    std::cout << "robot no charge !\n";
                    myrobot.octiBalanceStand();
                }
                break;
            default:
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    delete frame;

    myvideo.endWork();
    myvideo.watingDeviceEnding();
    while (1)
    {
        std::cout << "volum = " << (unsigned int)myrobot.getBatteryVolum() << std::endl;
        if (myrobot.getBatteryVolum() > highCharge)
        {
            myrobot.octiBalanceStand();
            break;
        }
        else
        {
            // 1min sample once Volum
            std::this_thread::sleep_for(std::chrono::seconds(60));
        }
    }
    myrobot.octiStandDown();
    std::cout << "finish charge\n";
    return 0;
}