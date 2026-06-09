#ifndef OCTI_ROBOT__HPP
#define OCTI_ROBOT__HPP
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unistd.h>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <eigen3/Eigen/Dense>
#include "path_control/include/json/json.hpp"
#include "path_control/include/localPlanner/local_planner.hpp"

namespace OCTIROBOT
{
    enum ROBOTERROR
    {
        SUCCSEE,
        ERROR,
    };
    enum ROBOTSPEEDLEVEL
    {
        LOW = -1,
        MIDDLE = 0,
        HIGH = 1,
    };
    enum ROBOTCONTROLLER
    {
        NAVIGATION,
        VOICEINTERACTION,
    };

    enum MANIPULATION
    {
        START_INSPECTION = 1,
        STOP_INSPECTION = 2,
        FORWARD = 3,
        BACKWARD = 4,
        TURN_LEFT = 5,
        TURN_RIGHT = 6,
        STAND_UP = 7,
        STAND_DOWN = 8,
        LEFT_MOVE = 9,
        RIGHT_MOVE = 16,
        STOP_MOTION,
        RECOVERY_MOTION,
    };

    enum OBJECTTYPE
    {
        SERVICER = 0,
        VISION = 1,
        ROBOT = 2,
        METER = 3,
        TEMPRATURE = 5,
        VOICEACTOR = 6,
    };

    struct commonFrameHead
    {
        uint32_t frameType;
        uint32_t source;
        uint32_t dest;
        uint32_t subObj;
    };

    struct commonFrameData
    {
        uint32_t hasData;
        uint32_t data1;
        uint32_t data2;
        float data3;
        float data4;
    };

    struct commonFrame
    {
        struct commonFrameHead frameHead;
        struct commonFrameData framedata;
    };



};

class octiRobot
{
private:
    unitree::robot::ChannelFactory *channelFactory = NULL;
    unitree::robot::go2::SportClient *octiRobotHandler = NULL;

    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> *lowstateSubscriber;
    unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_> *lowcmdPublisher;

    std::thread *motionInterfaceThreadHandler;

    const char *TOPIC_HIGHSTATE = "rt/sportmodestate";
    const char *TOPIC_LOWSTATE = "rt/lowstate";
    const char *TOPIC_LOWCMD = "rt/lowcmd";

private:
    std::mutex lowStateAccessMutex;
    unitree_go::msg::dds_::LowState_ lowStateMessage;
    std::atomic<bool> stopFlag;

private:
    unsigned int motionDelayUs;

private:
    OCTIROBOT::ROBOTCONTROLLER robotController; //who is controling robot
    std::atomic<bool> motionInterfaceEndFlag;
    std::mutex robotControllerMutex;

public:
    localPlannerBase *localPlanner;

private:
    void lowStateMessageHandler(const void *message);
    void motionInterfaceThreadFunc(int fd);

public:
    octiRobot(const char *networkInterface, localPlannerBase *localPlanner_, unsigned int motionDelayUs_, float timeout = 10.0f, uint32_t domain_id = 0);
    ~octiRobot();

    OCTIROBOT::ROBOTERROR octiStandUp();
    OCTIROBOT::ROBOTERROR octiStandDown();
    OCTIROBOT::ROBOTERROR octiBalanceStand();
    OCTIROBOT::ROBOTERROR octiEuler(float roll, float pitch, float yaw);
    OCTIROBOT::ROBOTERROR octiMove(double xVelocity, double yVelocity, double wVelocity);
    bool octiRotateToYaw(double nowYaw, double destYaw, double minErrorAngle);
    void octiStopLock();   // do not move, no matter if send move instruction
    void octiStopUnlock(); // unlock stop move

    void getRobotState(std::string stateType_, std::string &returnData);
    void getRobotState(std::string stateType_, double &returnData);
    void setMoveSpeedLevel(OCTIROBOT::ROBOTSPEEDLEVEL speedLevel);
    char getBatteryVolum();
    bool isBatteryCharging();

    //MotionInterface
    bool startMotionInterface(const char *motionInterfaceIp, unsigned short motionInterfacePort);
    bool endMotionInterface();
    void waitingMotionInterfaceEnd();

    OCTIROBOT::ROBOTCONTROLLER getRobotController(void);
    void setRobotController(OCTIROBOT::ROBOTCONTROLLER controllerType);
};

#endif