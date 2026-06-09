#ifndef OCTI_CHARGE__HPP
#define OCTI_CHARGE__HPP

#include <iostream>
#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>
#include <apriltag.h>
#include <tag36h11.h>
#include <common/homography.h>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace CHARGE
{
    struct aprilTagDataStruct
    {
        int tag_id;
        double point[4][2];
        double distance;
        double angle;
    };

    enum INSTRUCTION_TYPE
    {
        NONE,
        FINISH,
        MOVE,
        STANDUP,
        BALANCESTAND,
        STANDDOWN,
    };

    struct moveInstructionDataStruct
    {
        double xVelocity;
        double yVelocity;
        double wVelocity;
    };
}

class octiCharge
{
private:
    apriltag_family_t *apriltagFamilyInstacnce;
    apriltag_detector_t *apriltagDetectorInstacnce;

    float tagSize;
    cv::Mat frame;
    int changeFlag;
    int chargeEndFlag;

private:
    // 摄像头内参矩阵和畸变系数
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 813.3711625175978, 0, 641.4372246785323,
                            0, 811.9660350952695, 363.0425434004962,
                            0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -0.3782331371477318, 0.03993552130378619, 0.004878030381801512, -0.003346219709033664, 0.276745111025832);

private:
public:
    octiCharge(float tagSize_);
    ~octiCharge();

    void getCvMatData(cv::Mat matData);
    CHARGE::aprilTagDataStruct detectAprilTag();
    CHARGE::INSTRUCTION_TYPE chargeAction(CHARGE::aprilTagDataStruct aprilTagData_, CHARGE::moveInstructionDataStruct &moveInstructionData);
};

#endif