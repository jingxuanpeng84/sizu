#include "path_control/include/octi_robot.hpp"

octiRobot::octiRobot(const char *networkInterface, localPlannerBase *localPlanner_, unsigned int motionDelayUs_, float timeout, uint32_t domain_id)
{
    this->channelFactory = unitree::robot::ChannelFactory::Instance();
    this->channelFactory->Init(domain_id, networkInterface);
    this->localPlanner = localPlanner_;
    this->motionDelayUs = motionDelayUs_;
    this->stopFlag = false;
    this->robotController = OCTIROBOT::ROBOTCONTROLLER::NAVIGATION;
    this->octiRobotHandler = new unitree::robot::go2::SportClient(timeout);
    this->octiRobotHandler->Init();

    // robot state topic
    this->lowstateSubscriber = new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(this->TOPIC_LOWSTATE);
    this->lowstateSubscriber->InitChannel(std::bind(&octiRobot::lowStateMessageHandler, this, std::placeholders::_1), 1);
    this->lowcmdPublisher = new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(this->TOPIC_LOWCMD);
    this->lowcmdPublisher->InitChannel();
}

octiRobot::~octiRobot()
{
    if (this->octiRobotHandler != NULL)
    {
        delete this->octiRobotHandler;
    }
    if (this->lowstateSubscriber != NULL)
    {
        delete this->lowstateSubscriber;
    }
    if (this->lowcmdPublisher != NULL)
    {
        delete this->lowcmdPublisher;
    }
}

void octiRobot::setMoveSpeedLevel(OCTIROBOT::ROBOTSPEEDLEVEL speedLevel)
{
    this->octiRobotHandler->SpeedLevel(speedLevel);
}

/*
 * stateType_ #-> "state", "bodyHeight", "footRaiseHeight", "speedLevel", "gait","joystick", "dance", "continuousGait", "economicGait"
 *
 */

void octiRobot::getRobotState(std::string stateType_, std::string &returnData)
{
    std::string state;
    std::vector<std::string> state_name = {"speedLevel", "gait"};
    std::map<std::string, std::string> state_map;
    this->octiRobotHandler->GetState(state_name, state_map);
    nlohmann::json json_state = nlohmann::json::parse(state_map[stateType_.c_str()]);
    returnData = json_state["data"];
    return;
} 

/*
 * stateType_ #-> "state", "bodyHeight", "footRaiseHeight", "speedLevel", "gait", "joystick", "dance", "continuousGait", "economicGait"
 *
 */
void octiRobot::getRobotState(std::string stateType_, double &returnData)
{
    std::string state;
    std::vector<std::string> state_name = {"speedLevel", "gait"};
    std::map<std::string, std::string> state_map;
    this->octiRobotHandler->GetState(state_name, state_map);
    nlohmann::json json_state = nlohmann::json::parse(state_map[stateType_.c_str()]);
    returnData = json_state["data"];
    return;
}



OCTIROBOT::ROBOTERROR octiRobot::octiStandUp()
{
    // double robotState_;
    this->octiRobotHandler->StandUp();
    sleep(1);
    // do
    // {
    //     this->octiRobotHandler->StandUp();
    //     std::this_thread::sleep_for(std::chrono::microseconds(this->motionDelayUs));
    //     this->getRobotState("gait", robotState_);
    //     std::cout << "robot gait = " << robotState_ << std::endl;
    // } while (robotState_ != (-1) );
    return OCTIROBOT::ROBOTERROR::SUCCSEE;
}

OCTIROBOT::ROBOTERROR octiRobot::octiStandDown()
{
    // double robotState_;
    // this->octiStandUp();
    // sleep(1);
    this->octiRobotHandler->StandDown();
    // do
    // {
    //     this->octiRobotHandler->StandDown();
    //     std::this_thread::sleep_for(std::chrono::microseconds(this->motionDelayUs));
    //     this->getRobotState("gait", robotState_);
    // } while (robotState_ != -2 );
     return OCTIROBOT::ROBOTERROR::SUCCSEE;
}

OCTIROBOT::ROBOTERROR octiRobot::octiBalanceStand()
{
    // double robotState_;
    this->octiRobotHandler->BalanceStand();
    // do
    // {
    //     this->octiRobotHandler->BalanceStand();
    //     std::this_thread::sleep_for(std::chrono::microseconds(this->motionDelayUs));
    //     this->getRobotState("gait", robotState_);
    // } while (robotState_ != 0);
    return OCTIROBOT::ROBOTERROR::SUCCSEE;
}

OCTIROBOT::ROBOTERROR octiRobot::octiMove(double xVelocity, double yVelocity, double wVelocity)
{
    if (this->stopFlag != true)
    {
        if (this->octiRobotHandler->Move(xVelocity, yVelocity, wVelocity) == 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(this->motionDelayUs));
            return OCTIROBOT::ROBOTERROR::SUCCSEE;
        }
        else
        {
            return OCTIROBOT::ROBOTERROR::ERROR;
        }
    }
    return OCTIROBOT::ROBOTERROR::SUCCSEE;
}

/*
 * @brief octiRotateToYaw -> robot to dest yaw
 * @param nowYaw -> robot now yaw
 * @param destYaw -> destinational yaw
 * @param minErrorAngle -> the min thresh of error yaw
 * @return --> true: rotated dest yaw --> false: not rotate at dest yaw
 */
bool octiRobot::octiRotateToYaw(double nowYaw, double destYaw, double minErrorAngle)
{
    double errAngle = destYaw - nowYaw;
    if (errAngle > M_PI)
    {
        errAngle = errAngle - 2 * M_PI;
    }
    else if (errAngle < -M_PI)
    {
        errAngle = errAngle + 2 * M_PI;
    }
    // std::cout << "ro errAngle = " << errAngle << std::endl;
    if (abs(errAngle) < abs(minErrorAngle))
    {
        return true;
    }
    else
    {
        this->octiMove(0, 0, abs(errAngle) / errAngle * 0.5);
        return false;
    }
}

OCTIROBOT::ROBOTERROR octiRobot::octiEuler(float roll, float pitch, float yaw)
{
    std::string robotState_;
    this->octiBalanceStand();
    if (this->octiRobotHandler->Euler(roll, pitch, yaw) != 0)
    {
        return OCTIROBOT::ROBOTERROR::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(this->motionDelayUs));
    // this->getRobotState("state", robotState_);
    // std::cout << "euler state = " << robotState_ << std::endl;
    return OCTIROBOT::ROBOTERROR::SUCCSEE;
}

void octiRobot::octiStopLock()
{
    this->stopFlag = true;
    return;
}
void octiRobot::octiStopUnlock()
{
    this->stopFlag = false;
    return;
}

char octiRobot::getBatteryVolum()
{
    char batteryVolum = 0;
    this->lowStateAccessMutex.lock();
    batteryVolum = this->lowStateMessage.bms_state().soc();
    this->lowStateAccessMutex.unlock();
    return batteryVolum;
}

bool octiRobot::isBatteryCharging()
{
    this->lowStateAccessMutex.lock();
    if (this->lowStateMessage.bms_state().current() > 0)
    {
        this->lowStateAccessMutex.unlock();
        return true;
    }
    else
    {
        this->lowStateAccessMutex.unlock();
        return false;
    }
}

void octiRobot::lowStateMessageHandler(const void *message)
{
    this->lowStateAccessMutex.lock();
    this->lowStateMessage = (*(unitree_go::msg::dds_::LowState_ *)message);
    this->lowStateAccessMutex.unlock();
    return;
}

void octiRobot::motionInterfaceThreadFunc(int fd)
{
    sockaddr_in client;
    socklen_t size = sizeof(client);
    ssize_t num = 0;
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    // OCTIROBOT::motionFrame motionRecFrame;
    // std::cout << "motion start\n";
    OCTIROBOT::commonFrameData motionData;
    OCTIROBOT::commonFrame motionRecFrame;
    OCTIROBOT::commonFrameHead FrameHead;
    while (true)
    {
        if (this->motionInterfaceEndFlag == true)
        {
            close(fd);
            std::cout << "motionInterface close success\n";
            pthread_exit(NULL);
        }
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        //
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;
        int result = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (result < 0)
        {
            perror("select failed");
            close(fd);
            this->motionInterfaceEndFlag == true;
            pthread_exit(NULL);
        }
        else if (result == 0)
        {
            continue;
        }
        // receive client data
        num = recvfrom(fd, &motionRecFrame, sizeof(motionRecFrame), 0, (sockaddr *)&client, &size);
        // std::cout << "num = " << num << std::endl;
        if (num == sizeof(OCTIROBOT::commonFrame))
        {
            // std::cout << "rec data\n";
            FrameHead = motionRecFrame.frameHead;
            motionData = motionRecFrame.framedata;
            if(motionData.hasData != 1)
            {
                continue;
            }
            switch (FrameHead.source)
            {
            case OCTIROBOT::OBJECTTYPE::VOICEACTOR:
                /* code */
                if(this->getRobotController() == OCTIROBOT::ROBOTCONTROLLER::NAVIGATION)
                {
                    switch (motionData.data1)
                    {
                    case OCTIROBOT::MANIPULATION::STOP_INSPECTION:
                        this->setRobotController(OCTIROBOT::ROBOTCONTROLLER::VOICEINTERACTION);
                        break;
                    
                    default:
                        break;
                    }
                }
                else if(this->getRobotController() == OCTIROBOT::ROBOTCONTROLLER::VOICEINTERACTION)
                {
                    // std::cout << "voice = " << motionData.data1 << std::endl;
                    switch (motionData.data1)
                    {
                    case OCTIROBOT::MANIPULATION::STOP_INSPECTION:
                        this->setRobotController(OCTIROBOT::ROBOTCONTROLLER::VOICEINTERACTION);
                        break;
                    case OCTIROBOT::MANIPULATION::START_INSPECTION:
                        // this->octiStandUp();
                        this->octiBalanceStand();
                        this->setRobotController(OCTIROBOT::ROBOTCONTROLLER::NAVIGATION);
                        break;
                    case OCTIROBOT::MANIPULATION::FORWARD:
                        this->octiMove(0.2, 0, 0);
                        break;
                    case OCTIROBOT::MANIPULATION::BACKWARD:
                        this->octiMove(-0.2, 0, 0);
                        break;
                    case OCTIROBOT::MANIPULATION::TURN_LEFT:
                        this->octiMove(0, 0, 0.3);
                        break;
                    case OCTIROBOT::MANIPULATION::TURN_RIGHT:
                        this->octiMove(0, 0, -0.3);
                        break;
                    case OCTIROBOT::MANIPULATION::STAND_UP:
                        // this->octiStandUp();
                        // sleep(0.5);
                        this->octiBalanceStand();
                        break;
                    case OCTIROBOT::MANIPULATION::STAND_DOWN:
                        this->octiStandUp();
                        sleep(0.5);
                        this->octiStandDown();
                        break;
                    case OCTIROBOT::MANIPULATION::LEFT_MOVE:
                        this->octiMove(0, 0.2, 0);
                        break;
                    case OCTIROBOT::MANIPULATION::RIGHT_MOVE:
                        this->octiMove(0, -0.2, 0);
                        break;
                    default:
                        break;
                    }
                }
                break;
            case OCTIROBOT::OBJECTTYPE::SERVICER:
                /* code */
                if(this->getRobotController() == OCTIROBOT::ROBOTCONTROLLER::NAVIGATION)
                {
                    switch (motionData.data1)
                    {
                    case OCTIROBOT::MANIPULATION::STOP_INSPECTION:
                        this->setRobotController(OCTIROBOT::ROBOTCONTROLLER::VOICEINTERACTION);
                        break;
                    
                    default:
                        break;
                    }
                }
                else if(this->getRobotController() == OCTIROBOT::ROBOTCONTROLLER::VOICEINTERACTION)
                {
                    // std::cout << "server = " << motionData.data1 << std::endl;
                    switch (motionData.data1)
                    {
                    case OCTIROBOT::MANIPULATION::STOP_INSPECTION:
                        this->setRobotController(OCTIROBOT::ROBOTCONTROLLER::VOICEINTERACTION);
                        break;
                    case OCTIROBOT::MANIPULATION::START_INSPECTION:
                        // this->octiStandUp();
                        this->octiBalanceStand();
                        this->setRobotController(OCTIROBOT::ROBOTCONTROLLER::NAVIGATION);
                        break;
                    case OCTIROBOT::MANIPULATION::FORWARD:
                        this->octiMove(0.2, 0, 0);
                        break;
                    case OCTIROBOT::MANIPULATION::BACKWARD:
                        this->octiMove(-0.2, 0, 0);
                        break;
                    case OCTIROBOT::MANIPULATION::TURN_LEFT:
                        this->octiMove(0, 0, 0.3);
                        break;
                    case OCTIROBOT::MANIPULATION::TURN_RIGHT:
                        this->octiMove(0, 0, -0.3);
                        break;
                    case OCTIROBOT::MANIPULATION::STAND_UP:
                        this->octiBalanceStand();
                        break;
                    case OCTIROBOT::MANIPULATION::STAND_DOWN:
                        this->octiStandUp();
                        sleep(0.5);
                        this->octiStandDown();
                        break;
                    case OCTIROBOT::MANIPULATION::LEFT_MOVE:
                        this->octiMove(0, 0.2, 0);
                        break;
                    case OCTIROBOT::MANIPULATION::RIGHT_MOVE:
                        this->octiMove(0, -0.2, 0);
                        break;
                    default:
                        break;
                    }
                }
                break;
            default:
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }
    }
}

bool octiRobot::startMotionInterface(const char *motionInterfaceIp, unsigned short motionInterfacePort)
{
    this->motionInterfaceEndFlag = false;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::cout << "sock error\n";
        this->motionInterfaceEndFlag = true;
        return false;
    }
    sockaddr_in mySockAddrIn;
    memset(&mySockAddrIn, 0, sizeof(mySockAddrIn));
    mySockAddrIn.sin_family = AF_INET;
    mySockAddrIn.sin_port = htons(motionInterfacePort);
    mySockAddrIn.sin_addr.s_addr = inet_addr(motionInterfaceIp);
    int ret = bind(fd, (sockaddr *)&mySockAddrIn, sizeof(mySockAddrIn));
    if (ret < 0)
    {
        std::cout << "bind error\n";
        close(fd);
        this->motionInterfaceEndFlag = true;
        return false;
    }
    this->motionInterfaceThreadHandler = new std::thread(std::bind(&octiRobot::motionInterfaceThreadFunc, this, fd));
    if (this->motionInterfaceThreadHandler == NULL)
    {
        std::cout << "thread start error\n";
        close(fd);
        this->motionInterfaceEndFlag = true;
        return false;
    }
    std::cout << "motionInterface open success!\n";
    return true;
}
bool octiRobot::endMotionInterface()
{
    this->motionInterfaceEndFlag = true;
    this->waitingMotionInterfaceEnd();
    return true;
}
void octiRobot::waitingMotionInterfaceEnd()
{
    if (this->motionInterfaceThreadHandler != NULL)
    {
        this->motionInterfaceThreadHandler->join();
        this->motionInterfaceThreadHandler = NULL;
    }
}

OCTIROBOT::ROBOTCONTROLLER octiRobot::getRobotController(void)
{
    OCTIROBOT::ROBOTCONTROLLER controllerTypeTemp;
    this->lowStateAccessMutex.lock();
    controllerTypeTemp = this->robotController;
    this->lowStateAccessMutex.unlock();
    return controllerTypeTemp;
}
void octiRobot::setRobotController(OCTIROBOT::ROBOTCONTROLLER controllerType)
{
    this->lowStateAccessMutex.lock();
    this->robotController = controllerType;
    this->lowStateAccessMutex.unlock();
    return;
}