#include <string>
#include "path_control/include/localPlanner/octi_dwa.hpp"
#include "path_control/include/laddar_device.hpp"
#include "path_control/include/vision_device.hpp"
#include "path_control/include/video_device.hpp"
#include "path_control/include/octi_charge.hpp"
#include "path_control/include/octi_robot.hpp"
#include "path_control/include/octi_xml.hpp"
#include "path_control/include/robotShowUi/ui.hpp"
#include "path_control/include/octi_yaml.hpp"
#include "path_control/include/robot_elevator_client.hpp"
#include "path_control/include/send.hpp"


using namespace std;
using namespace Eigen;

namespace UTILITY
{
#define LINE_NUM 181
    struct obstacleOriginDataStruct
    {
        double x;
        double y;
        double yaw;
        double dist[LINE_NUM];
    };
    double computerDAngle(Eigen::VectorXd robotState, Eigen::Vector2d goal);
    bool getObstacle(std::vector<Eigen::Vector2d> &obstacle, obstacleOriginDataStruct obstacleOriginData, std::pair<double, double> &distRange, double detaAngle);
};

double UTILITY::computerDAngle(Eigen::VectorXd robotState, Eigen::Vector2d goal)
{
    double d_angle = atan2(goal[1] - robotState[1], goal[0] - robotState[0]);
    double err_angle = d_angle - robotState[2];
    if (err_angle > M_PI)
    {
        err_angle = err_angle - 2 * M_PI;
    }
    else if (err_angle < -M_PI)
    {
        err_angle = err_angle + 2 * M_PI;
    }
    return err_angle;
}

bool UTILITY::getObstacle(std::vector<Eigen::Vector2d> &obstacle, obstacleOriginDataStruct obstacleOriginData, std::pair<double, double> &distRange, double detaAngle)
{
    obstacle.clear();
    Eigen::VectorXd robotState(5);
    robotState[0] = obstacleOriginData.x;
    robotState[1] = obstacleOriginData.y;
    robotState[2] = obstacleOriginData.yaw;
    // std::cout << "gto en\n";
    for (unsigned long i = 0; i <= sizeof(obstacleOriginData.dist) / sizeof(double); ++i)
    {
        if (obstacleOriginData.dist[i] <= distRange.second && obstacleOriginData.dist[i] >= distRange.first)
        {
            Eigen::Vector2d coordination;
            // std::cout << "compute enter\n";
            // robot yaw > 0 or yaw < 0, the compute is the same
            coordination[0] = robotState[0] + obstacleOriginData.dist[i] * cos((i * detaAngle * M_PI / 180.0) - M_PI / 2 - robotState[2]);
            coordination[1] = (robotState[1] + obstacleOriginData.dist[i] * sin(-((i * detaAngle * M_PI / 180.0) - M_PI / 2 - robotState[2])));
            obstacle.emplace_back(coordination);
        }
    }
    return true;
}

int main()
{
    std::cout << "Loading config.yaml\n";
    octi_yaml yaml("/home/orangepi/Desktop/work_station/OctiRobotVersion2/path_control/target/config/config.yaml");
    std::string robotNetInterface = "nan";
    uint32_t robotActionDurationTime = 0;
    std::string reversalPathXmlPath = "nan";

    uint32_t currentGoalIndexRecordLoad = 0;
    std::string currentGoalIndexXmlPath = "nan";

    std::string laddarIp = "nan";
    uint32_t laddarPort = 0;
    uint32_t map_switch_PORT = 0;

    std::string visionIp = "nan";
    std::string robotVisionIp = "nan";
    uint32_t visionPort = 0;
    uint32_t visionLoad = 0;
    uint32_t robotVisionPort = 0;

    uint32_t autoChargeLoad = 0;
    uint32_t lowCharge = 0;
    uint32_t highCharge = 0;

    std::string myIp = "nan";
    uint32_t myPort = 0;

    std::string motionInterfaceIp = "nan";
    uint32_t motionInterfacePort = 0;
    uint32_t motionInterfaceLoad = 0;

    std::string videoDeviceAddr = "nan";
    uint32_t videoDeviceLoad = 0;

    std::string speakerIp = "nan";
    uint32_t speakerPort = 0;
    uint32_t speakerLoad = 0;

    uint32_t uiShowLoad = 0;

    std::string localPlanner = "nan";
    // dwa
    double sampleDetaTime;
    double vMin;
    double vMax;
    double wMin;
    double wMax;
    double aVMax;
    double aWMax;
    double vSampleDetaV;
    double wSampleDetaW;
    double predictTime;
    double alpha;
    double beta;
    double gamma;
    double keyPointRadius;
    double robot_radius;

    localPlannerBase *localPlannerIns = NULL;

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

    // motionInterface
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

    // load reversalPathXmlPath
    if (yaml.IsDefine("reversalPathXmlPath") == false)
    {
        perror("target/config/config.yaml: Please define reversalPathXmlPath !");
        exit(1);
    }
    reversalPathXmlPath = yaml.getString("reversalPathXmlPath");
    std::cout << "reversalPathXmlPath = " << reversalPathXmlPath << std::endl;

    // load currentGoalIndexRecord
    if (yaml.IsDefine("currentGoalIndexRecordLoad") == true)
    {
        if (yaml.getUint("currentGoalIndexRecordLoad") == 1)
        {
            if (yaml.IsDefine("currentGoalIndexXmlPath") == false)
            {
                perror("target/config/config.yaml: Please define currentGoalIndexXmlPath, when need load currentGoalIndexRecord !");
                exit(1);
            }
            currentGoalIndexXmlPath = yaml.getString("currentGoalIndexXmlPath");
            currentGoalIndexRecordLoad = 1;
            std::cout << "currentGoalIndexXmlPath = " << currentGoalIndexXmlPath << std::endl;
        }
    }

    // load laddar
    if (yaml.IsDefine("laddarIp") == false || yaml.IsDefine("laddarPort") == false || yaml.IsDefine("map_switch_PORT") == false)
    {
        perror("target/config/config.yaml: Please define laddarIp / laddarPort / map_switch_PORT, when need load laddar !");
        exit(1);
    }
    laddarIp = yaml.getString("laddarIp");
    laddarPort = yaml.getUint("laddarPort");
    map_switch_PORT = yaml.getUint("map_switch_PORT");
    std::cout << "laddarIp = " << laddarIp << " laddarPort = " << laddarPort << " map_switch_PORT = " << map_switch_PORT << std::endl;

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

    // load my ip and my port
    if (yaml.IsDefine("myIp") == false || yaml.IsDefine("myPort") == false)
    {
        perror("target/config/config.yaml: Please define myIp / myPort !");
        exit(1);
    }
    myIp = yaml.getString("myIp");
    myPort = yaml.getUint("myPort");
    std::cout << "myIp = " << myIp << " myPort = " << myPort << std::endl;

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

    // load speaker
    if (yaml.IsDefine("speakerLoad") == true)
    {
        if (yaml.getUint("speakerLoad") == 1)
        {
            if (yaml.IsDefine("speakerIp") == false || yaml.IsDefine("speakerPort") == false)
            {
                perror("target/config/config.yaml: Please define speakerIp / speakerPort, when need load speaker !");
                exit(1);
            }
            speakerLoad = 1;
            laddarIp = yaml.getString("speakerIp");
            laddarPort = yaml.getUint("speakerPort");
            std::cout << "speakerIp = " << speakerIp << " speakerPort = " << speakerPort << std::endl;
        }
    }

    // load uiShow
    if (yaml.IsDefine("uiShowLoad") == true)
    {
        uiShowLoad = yaml.getUint("uiShowLoad");
        std::cout << "uiShowLoad = " << uiShowLoad << std::endl;
    }

    // local planner
    if (yaml.IsDefine("localPlanner") == false)
    {
        perror("target/config/config.yaml: Please define localPlanner !");
        exit(1);
    }
    localPlanner = yaml.getString("localPlanner");
    std::cout << "localPlanner = " << localPlanner << std::endl;
    if (localPlanner == "DWA")
    {
        if (yaml.IsDefine("sampleDetaTime") == false ||
            yaml.IsDefine("vMin") == false ||
            yaml.IsDefine("vMax") == false ||
            yaml.IsDefine("wMin") == false ||
            yaml.IsDefine("wMax") == false ||
            yaml.IsDefine("aVMax") == false ||
            yaml.IsDefine("aWMax") == false ||
            yaml.IsDefine("vSampleDetaV") == false ||
            yaml.IsDefine("wSampleDetaW") == false ||
            yaml.IsDefine("predictTime") == false ||
            yaml.IsDefine("alpha") == false ||
            yaml.IsDefine("beta") == false ||
            yaml.IsDefine("gamma") == false ||
            yaml.IsDefine("keyPointRadius") == false ||
            yaml.IsDefine("robot_radius") == false)
        {
            perror("target/config/config.yaml: Please define DWA parameters when need load DWA local planner !");
            exit(1);
        }
        sampleDetaTime = yaml.getDouble("sampleDetaTime");
        vMin = yaml.getDouble("vMin");
        vMax = yaml.getDouble("vMax");
        wMin = yaml.getDouble("wMin");
        wMax = yaml.getDouble("wMax");
        aVMax = yaml.getDouble("aVMax");
        aWMax = yaml.getDouble("aWMax");
        vSampleDetaV = yaml.getDouble("vSampleDetaV");
        wSampleDetaW = yaml.getDouble("wSampleDetaW");
        predictTime = yaml.getDouble("predictTime");
        alpha = yaml.getDouble("alpha");
        beta = yaml.getDouble("beta");
        gamma = yaml.getDouble("gamma");
        keyPointRadius = yaml.getDouble("keyPointRadius");
        robot_radius = yaml.getDouble("robot_radius");
        std::cout
            << "DWA parameters : -> "
            << " sampleDetaTime = " << sampleDetaTime
            << " vMin = " << vMin
            << " vMax = " << vMax
            << " wMin = " << wMin
            << " wMax = " << wMax
            << " aVMax = " << aVMax
            << " aWMax = " << aWMax
            << " vSampleDetaV = " << vSampleDetaV
            << " wSampleDetaW = " << wSampleDetaW
            << " predictTime = " << predictTime
            << " alpha = " << alpha
            << " beta = " << beta
            << " gamma = " << gamma
            << " keyPointRadius = " << keyPointRadius
            << " robot_radius = " << robot_radius << std::endl;
    }

    std::cout << "start!\n";
    std::vector<std::thread *> threadVector;


 

    //laddar init
    deviceBase *laddar = new laddarDevice(laddarIp.c_str(), laddarPort, myIp.c_str(), myPort);
    if (laddar->startWork() == false)
    {
        perror("Laddar error");
        exit(1);
    }
    // waiting laddar begin
    while (!laddar->isDeviceBeginWorking())
    {
        // std::cout << "waiting laddar\n";
    }

    // vision
    deviceBase *myVision = NULL;
    if (visionLoad == 1)
    {
        myVision = new visionDevice(visionIp.c_str(), visionPort, robotVisionIp.c_str(), robotVisionPort);
        if (myVision->startWork() == false)
        {
            perror("Vision start error");
            // exit(0);
        }
    }

    // video
    videoDevice *myvideo = NULL;
    if (videoDeviceLoad == 1)
    {
        myvideo = new videoDevice(videoDeviceAddr.c_str());
        // if (myvideo->startWork() == false)
        // {
        //     perror("Video start error");
        //     exit(0);
        // }
        // else
        // {
        //     // open window. or not
        //     myvideo->openWindowShow();
        // }
    }

    // autoCharge
    octiCharge *mycharge = NULL;
    if (autoChargeLoad == 1 && videoDeviceLoad == 1 && myvideo != NULL)
    {
        mycharge = new octiCharge(0.082);
    }

    // // ui
    uidata uihandle;
    if (uiShowLoad == 1)
    {
        std::thread *ui_thread = new std::thread(robot_ui_thread, &uihandle);
        threadVector.push_back(ui_thread);
    }

    // #begin -> read path
    octiXml myxml;
    std::vector<OCTIXML::pathNodeStruct> pathVector = myxml.readPath(reversalPathXmlPath.c_str());

    LOCAL_PLANNER::pathNode localPlanPathNode;

    unsigned long path_index_now = 0;
    if (currentGoalIndexRecordLoad == 1)
    {
        // read xml currentGoalIndexXmlPath
        // change path_index_now
    }

    // #begin -> dwa init
    double judge_distance = 10; // 若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
    if (localPlanner == "DWA")
    {
        if (pathVector.size() > 0)
        {
            localPlanPathNode.x = pathVector[path_index_now].x;
            localPlanPathNode.y = pathVector[path_index_now].y;
            localPlanPathNode.yaw = pathVector[path_index_now].yaw;
        }
        localPlannerIns = new DWA(localPlanPathNode, sampleDetaTime, vMin, vMax, wMin, wMax, predictTime, aVMax, aWMax, vSampleDetaV, wSampleDetaW * M_PI / 180, alpha, beta, gamma, keyPointRadius, judge_distance, robot_radius);
    }
    // #end -> dwa init

    // create motion_control
    octiRobot myrobot = octiRobot(robotNetInterface.c_str(), localPlannerIns, robotActionDurationTime);
    myrobot.startMotionInterface(motionInterfaceIp.c_str(), motionInterfacePort);
    // myrobot.octiStandUp();
    // myrobot.octiBalanceStand();

    // creat elevator_control
    // RobotElevatorClient myrobotelevatorclient("192.168.110.221", 8000, 1);
    // // RobotElevatorClient myrobotelevatorclient("/dev/ttyUSB2", 9600, 'N', 8, 1, 1);
    // req_frame request;


    VectorXd state(5); //[x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    double d_radius = 0.0;

    vector<Vector2d> obstacle; // 障碍物位置

    //-------------------------begin while-----------------------------//
    while (true)
    {
        // input state / goal / obstacle
        LADDAR::dataStruct laddarData;
        unsigned int laddarDataSize = 0;
        if (laddar->readDeviceData(&laddarData, laddarDataSize) == false)
        {
            std::cout << "Laddar may disconnect! Please check!\n";
            break;
        }

        // #begin -> update laddar data
        state[0] = laddarData.x;
        state[1] = laddarData.y;
        state[2] = laddarData.yaw;

        //-----------------------------------
        std::pair<double, double> distRange(0.15, 2);
        UTILITY::getObstacle(obstacle, *((UTILITY::obstacleOriginDataStruct *)&laddarData), distRange, 1);
        //---------------------------------------------------------------
        // #end -> update laddar data

        // // #begin -> ui render
        // if (uiShowLoad == 1)
        // {
        //     uihandle.setObsRobotData(obstacle, state);
        // }
        // // #end -> ui render

        // --------------------- judge the controller of robot ------------------------ //
        if (myrobot.getRobotController() == OCTIROBOT::ROBOTCONTROLLER::NAVIGATION)
        {
            // if back to destPoint, then rotate forward to destPoint
            Eigen::Vector2d goal_temp(pathVector[path_index_now].x, pathVector[path_index_now].y);
            if (abs(UTILITY::computerDAngle(state, goal_temp)) >= 3 * M_PI / 4 && (goal_temp - state.head(2)).norm() > keyPointRadius)
            {
                std::cout << "Rotate\n";
                // std::cout << "errorAngle = " << abs(UTILITY::computerDAngle(state, goal_temp)) << " nowYaw = " << state[2] << std::endl;
                double destAngle = atan2(pathVector[path_index_now].y - state[1], pathVector[path_index_now].x - state[0]);
                while (myrobot.octiRotateToYaw(laddarData.yaw, destAngle, M_PI / 8) != true)
                {
                    // std::cout << "Rotate\n";
                    if (laddar->readDeviceData(&laddarData, laddarDataSize) == false)
                    {
                        std::cout << "Laddar may disconnect! Please check!\n";
                        perror("laddar error\n");
                        exit(0);
                        break;
                    }
                }
                state[0] = laddarData.x;
                state[1] = laddarData.y;
                state[2] = laddarData.yaw;
                state[3] = 0;
                state[4] = 0;
                continue;
            }
            

            // #begin -> dwa controler
            LOCAL_PLANNER::plannerResult res = myrobot.localPlanner->plan(state[0], state[1], state[2], state[3], state[4], obstacle);
            // #end -> dwa controler

            if (res.dwaReturnType == LOCAL_PLANNER::DWARETURNTYPE::ARRIVALDEST)
            {
                
                commonFrame sendpoint;
                sendpoint.frameHead.frameType = 3;
                sendpoint.frameHead.source = 1;
                sendpoint.frameHead.dest = 0;
                sendpoint.frameHead.subObj = 1;

                sendpoint.frameData.hasData = 1; 
                sendpoint.frameData.data1 = pathVector[path_index_now].sequence;
                sendpoint.frameData.data2 = pathVector[path_index_now].keyPointType;
                sendpoint.frameData.data3 = pathVector[path_index_now].x;
                sendpoint.frameData.data4 = pathVector[path_index_now].y;
                sendPoint(sendpoint);

                sendpoint.frameHead.frameType = 3;
                sendpoint.frameHead.source = 1;
                sendpoint.frameHead.dest = 1;
                sendpoint.frameHead.subObj = 1;

                sendpoint.frameData.hasData = 1; 
                sendpoint.frameData.data1 = 1;
                sendpoint.frameData.data2 = 1;
                sendpoint.frameData.data3 = get_utc_timestamp();
                sendpoint.frameData.data4 = get_utc_timestamp();
                sendPoint(sendpoint);



                // std::cout << "arrival at point\n";
                switch (pathVector[path_index_now].keyPointType)
                {
                case OCTIXML::KEYPONITTYPE::NORMAL:
                    break;
                case OCTIXML::KEYPONITTYPE::VISION:

                    if (visionLoad == 1)
                    {
                        std::cout << "Vision point arrival\n";
                        //---rotate to Vision point
                        // while (myrobot.octiRotateToYaw(laddarData.yaw, pathVector[path_index_now].yaw, 8 * M_PI / 180.0) != true)
                        // {
                        //     if (laddar->readDeviceData(&laddarData, laddarDataSize) == false)
                        //     {
                        //         std::cout << "Laddar may disconnect! Please check!\n";
                        //         perror("laddar error\n");
                        //         exit(0);
                        //         break;
                        //     }
                        // }
                        //
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
                        // myrobot.octiEuler(0, -0.3, 0);
                        while (true)
                        {
                            if (myVision->readDeviceData(&visionRecData, visionSize) == false)
                            {
                                std::cout << "Vision meter may disconnect! Please check!\n";
                                // myrobot.octiEuler(0, 0, 0);
                                break; // finish vision
                            }
                            else if (visionRecData.instruction == VISION::instructionTypeEnum::ACK)
                            {
                                // myrobot.octiEuler(0, -0.35, 0);
                                std::this_thread::sleep_for(std::chrono::microseconds(200));
                                visionData.instruction = VISION::instructionTypeEnum::ALIVE;
                                myVision->sendDeviceData(&visionData, visionSize);
                            }
                            else if (visionRecData.instruction == VISION::instructionTypeEnum::FINAL)
                            {
                                std::cout << "Meter final\n";
                                std::this_thread::sleep_for(std::chrono::microseconds(500));
                                // myrobot.octiEuler(0, 0, 0);
                                break; // finish vision
                            }
                            else
                            {
                                std::this_thread::sleep_for(std::chrono::microseconds(500));
                                // myrobot.octiEuler(0, 0, 0);
                                break; // finish vision
                            }
                        }
                        // if()
                        //-------------------------
                    }
                    break;
                case OCTIXML::KEYPONITTYPE::CHARGE:
                    if (autoChargeLoad == 1 && mycharge != NULL && videoDeviceLoad == 1 && myrobot.getBatteryVolum() < lowCharge)
                    {
                        std::cout << "enter charge\n";
                        if (myvideo->startWork() == true)
                        {
                            // myvideo->openWindowShow();
                            //---rotate to charge point
                            while (myrobot.octiRotateToYaw(laddarData.yaw, pathVector[path_index_now].yaw, 4 * M_PI / 180.0) != true)
                            {
                                if (laddar->readDeviceData(&laddarData, laddarDataSize) == false)
                                {
                                    std::cout << "Laddar may disconnect! Please check!\n";
                                    perror("laddar error\n");
                                    exit(0);
                                    break;
                                }
                            }
                            //-------------------------
                            cv::Mat *frameCharge = new cv::Mat();
                            unsigned int dataSize;
                            // myrobot.octiStandUp();
                            myrobot.setMoveSpeedLevel(OCTIROBOT::ROBOTSPEEDLEVEL::LOW);
                            bool chargeFlag = false;
                            int try_times = 0;
                            while (!chargeFlag)
                            {
                                if (myvideo->readDeviceData(frameCharge, dataSize))
                                {
                                    CHARGE::aprilTagDataStruct chargeData;
                                    try_times = 0;
                                    do
                                    {
                                        mycharge->getCvMatData(*frameCharge);
                                        chargeData = mycharge->detectAprilTag();
                                        if (chargeData.tag_id >= 0)
                                        {
                                            break;
                                        }
                                        else
                                        {
                                            try_times++;
                                            if (try_times >= 6)
                                            {
                                                break;
                                            }
                                        }
                                        myvideo->readDeviceData(frameCharge, dataSize);
                                    } while (1);
                                    CHARGE::moveInstructionDataStruct chargeMoveData;
                                    CHARGE::INSTRUCTION_TYPE insType = mycharge->chargeAction(chargeData, chargeMoveData);
                                    switch (insType)
                                    {
                                    case CHARGE::INSTRUCTION_TYPE::MOVE:
                                        // std::cout << "V = " << chargeMoveData.xVelocity << " " << chargeMoveData.yVelocity << " " << chargeMoveData.wVelocity << std::endl;
                                        myrobot.octiMove(chargeMoveData.xVelocity, chargeMoveData.yVelocity, chargeMoveData.wVelocity);
                                        break;
                                    case CHARGE::INSTRUCTION_TYPE::STANDDOWN:
                                        // myrobot.octiMove(0,0,0.1);
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
                            }
                            delete frameCharge;
                            myvideo->endWork();
                            myvideo->watingDeviceEnding();
                            // charging check
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
                                    // 70s sample once Volum
                                    std::this_thread::sleep_for(std::chrono::seconds(70));
                                }
                            }
                        }
                    }
                    else
                    {
                        std::cout << "Can not charge\n";
                    }
                    break;
                // case OCTIXML::KEYPONITTYPE::WAIT:
                // {
                //     double Angle = 0;
                //     do
                //     {
                //         if (laddar->readDeviceData(&laddarData, laddarDataSize) == false)
                //         {
                //             std::cout << "Laddar may disconnect! Please check!\n";
                //             break;
                //         }
                //         // #begin -> update laddar data
                //         state[0] = laddarData.x;
                //         state[1] = laddarData.y;
                //         state[2] = laddarData.yaw;
                //         Eigen::Vector2d goal_temp(pathVector[path_index_now + 1].x, pathVector[path_index_now + 1].y);
                //         if (abs(UTILITY::computerDAngle(state, goal_temp)) <= M_PI / 8)
                //         {
                //             break;
                //         }
                //         Angle = atan2(pathVector[path_index_now + 1].y - laddarData.y, pathVector[path_index_now + 1].x - laddarData.x);
                //     } while (myrobot.octiRotateToYaw(laddarData.yaw, Angle, 4 * M_PI / 180.0) != true);

                //     do 
                //     {
                //        myrobot.octiBalanceStand();
                //     }
                //     while (myrobotelevatorclient.callElevatorAndOpenDoor(pathVector[path_index_now].mapnum) != true);
                //     break;
                // }
                // case OCTIXML::KEYPONITTYPE::INELEVATOR:
                // {
                //     double Angle = 0;
                //     do
                //     {
                //         if (laddar->readDeviceData(&laddarData, laddarDataSize) == false)
                //         {
                //             std::cout << "Laddar may disconnect! Please check!\n";
                //             break;
                //         }
                //         // #begin -> update laddar data
                //         state[0] = laddarData.x;
                //         state[1] = laddarData.y;
                //         state[2] = laddarData.yaw;
                //         Eigen::Vector2d goal_temp(pathVector[path_index_now + 1].x, pathVector[path_index_now + 1].y);
                //         if (abs(UTILITY::computerDAngle(state, goal_temp)) <= M_PI / 8)
                //         {
                //             break;
                //         }
                //         Angle = atan2(pathVector[path_index_now + 1].y - laddarData.y, pathVector[path_index_now + 1].x - laddarData.x);
                //     } while (myrobot.octiRotateToYaw(laddarData.yaw, Angle, 4 * M_PI / 180.0) != true);
                //     do 
                //     {
                //        myrobot.octiBalanceStand();
                //     }
                //     while (myrobotelevatorclient.rideToTargetFloorAndOpenDoor(pathVector[path_index_now + 1].mapnum) != true);
                //     break;
                // }
                // case OCTIXML::KEYPONITTYPE::MAPSWITCH:
                //     request.frame_type = pathVector[path_index_now].mapnum; //后续根据xml文件得目标点地图编号更改
                //     request.seq = 1;
                //     request.x = state[0];
                //     request.y = state[1];
                //     request.yaw = state[2];
                //     do 
                //     {
                //        myrobot.octiBalanceStand();
                //     }
                //     while (myrobotelevatorclient.closeDoorAndSwitchMap(request, laddarIp.c_str(), map_switch_PORT) != true);
                //     break;
                default:
                    perror("Unrecognized point type");
                    exit(0);
                    break;
                }
                //------------------change next dest point---------------------//
                path_index_now = (path_index_now + 1) % pathVector.size();
                localPlanPathNode.x = pathVector[path_index_now].x;
                localPlanPathNode.y = pathVector[path_index_now].y;
                localPlanPathNode.yaw = pathVector[path_index_now].yaw;
                myrobot.localPlanner->setDestinationPoint(localPlanPathNode);
                // std::cout << "next point " << localPlanPathNode.x << " " << localPlanPathNode.y << std::endl;
                //------------------Rotote to next point------------------------//
                double destAngle = 0;
                do
                {
                    if (laddar->readDeviceData(&laddarData, laddarDataSize) == false)
                    {
                        std::cout << "Laddar may disconnect! Please check!\n";
                        break;
                    }
                    // #begin -> update laddar data
                    state[0] = laddarData.x;
                    state[1] = laddarData.y;
                    state[2] = laddarData.yaw;
                    Eigen::Vector2d goal_temp(pathVector[path_index_now].x, pathVector[path_index_now].y);
                    if (abs(UTILITY::computerDAngle(state, goal_temp)) <= M_PI / 8)
                    {
                        break;
                    }
                    destAngle = atan2(localPlanPathNode.y - laddarData.y, localPlanPathNode.x - laddarData.x);
                } while (myrobot.octiRotateToYaw(laddarData.yaw, destAngle, 4 * M_PI / 180.0) != true);
                //--------------------------------------------------------------//
                state[3] = 0;
                state[4] = 0;
                continue; // go to next local planner
                //--------------------------------------------------------------//
            }
            // #begin -> robot move
            std::cout << "v == " << res.xV << " " << res.wV << std::endl;
            if (abs(res.xV) >= 0.05 || abs(res.yV) >= 0.05 || abs(res.wV) >= 0.05)
            {
                if (res.xV < 0.05)
                {
                    res.xV = 0;
                }
                myrobot.octiMove(res.xV, res.yV, res.wV); // v_x, v_y, v_w
                state[3] = res.xV;
                state[4] = res.wV;
            }
            else
            {
                state[3] = 0;
                state[4] = 0;
            }
            // #end -> robot move
        }
    } // end while

     myrobot.endMotionInterface();

    if (localPlannerIns != NULL)
    {
        delete localPlannerIns;
    }
    if (myVision != NULL)
    {
        myVision->endWork();
        myVision->watingDeviceEnding();
        // delete myVision;
        myVision = NULL;
    }
    if (videoDeviceLoad == 1 && myvideo != NULL)
    {
        myvideo->endWork();
        myvideo->watingDeviceEnding();
        // delete myvideo;
        myvideo = NULL;
    }
    // // #begin -> exit
    for (std::thread *thread : threadVector)
    {
        if (thread->joinable())
        {
            thread->join();
        }
    }
    laddar->endWork();
    laddar->watingDeviceEnding();
    //delete laddar;

    return 0;
}