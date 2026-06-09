#include "path_control/include/octi_charge.hpp"
octiCharge::octiCharge(float tagSize_)
{
    // create AprilTag detector
    this->apriltagFamilyInstacnce = tag36h11_create();
    this->apriltagDetectorInstacnce = apriltag_detector_create();
    apriltag_detector_add_family(this->apriltagDetectorInstacnce, this->apriltagFamilyInstacnce);
    this->tagSize = tagSize_;
    this->changeFlag = 0;
    this->chargeEndFlag = 0;
}

octiCharge::~octiCharge()
{
    if (this->apriltagFamilyInstacnce != NULL)
    {
        tag36h11_destroy(this->apriltagFamilyInstacnce);
    }
    if (this->apriltagDetectorInstacnce != NULL)
    {
        apriltag_detector_destroy(this->apriltagDetectorInstacnce);
    }
}

CHARGE::INSTRUCTION_TYPE octiCharge::chargeAction(CHARGE::aprilTagDataStruct aprilTagData_, CHARGE::moveInstructionDataStruct &moveInstructionData)
{
    if (this->chargeEndFlag == 1)
    {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        this->chargeEndFlag = 0;
        return CHARGE::INSTRUCTION_TYPE::FINISH;
    }
    else
    {
        this->changeFlag = 0;
        if (aprilTagData_.tag_id < 0)
        {
            moveInstructionData.xVelocity = 0;
            moveInstructionData.yVelocity = 0;
            moveInstructionData.wVelocity = -0.3;
            return CHARGE::INSTRUCTION_TYPE::MOVE;
        }
        // turn around
        if (abs(aprilTagData_.angle * 180.0 / CV_PI) > 4)
        {
            moveInstructionData.xVelocity = 0;
            moveInstructionData.yVelocity = 0;
            std::cout << "v_a = " << -aprilTagData_.angle * 1.2 << std::endl;
            moveInstructionData.wVelocity = -aprilTagData_.angle * 1.2;
            this->changeFlag = 1;
            return CHARGE::INSTRUCTION_TYPE::MOVE;
        }
        float left = aprilTagData_.point[0][1] - aprilTagData_.point[3][1];
        float right = aprilTagData_.point[1][1] - aprilTagData_.point[2][1];
        if (abs(left - right) > 0.7)
        {
            if (left > right)
            {
                moveInstructionData.xVelocity = 0;
                moveInstructionData.yVelocity = -0.1;
                moveInstructionData.wVelocity = 0;
            }
            else if (left < right)
            {
                moveInstructionData.xVelocity = 0;
                moveInstructionData.yVelocity = 0.1;
                moveInstructionData.wVelocity = 0;
            }
            this->changeFlag = 1;
            return CHARGE::INSTRUCTION_TYPE::MOVE;
        }
        if (abs(aprilTagData_.distance - 0.47) > 0.08)
        {
            if (aprilTagData_.distance - 0.47 > 0)
            {
                moveInstructionData.xVelocity = 0.1;
                moveInstructionData.yVelocity = 0;
                moveInstructionData.wVelocity = 0;
            }
            else if (aprilTagData_.distance - 0.47 < 0)
            {
                moveInstructionData.xVelocity = -0.1;
                moveInstructionData.yVelocity = 0;
                moveInstructionData.wVelocity = 0;
            }
            this->changeFlag = 1;
            return CHARGE::INSTRUCTION_TYPE::MOVE;
        }
        if (this->changeFlag == 0)
        {
            this->chargeEndFlag = 1;
            return CHARGE::INSTRUCTION_TYPE::STANDDOWN;
        }
    }
    return CHARGE::INSTRUCTION_TYPE::NONE;
}

void octiCharge::getCvMatData(cv::Mat matData)
{
    this->frame = matData;
    return;
}

CHARGE::aprilTagDataStruct octiCharge::detectAprilTag()
{
    cv::Mat grayFrame;
    CHARGE::aprilTagDataStruct returnData;
    cv::cvtColor(this->frame, grayFrame, cv::COLOR_BGR2GRAY);
    image_u8_t im = {.width = grayFrame.cols,
                     .height = grayFrame.rows,
                     .stride = grayFrame.cols,
                     .buf = grayFrame.data};
    zarray_t *detections = apriltag_detector_detect(this->apriltagDetectorInstacnce, &im);
    returnData.tag_id = -1; //no detect tag, else return tagId will be 0,this may mistake
    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        //----------window show-----------
        // draw bondary of detected tag
        cv::line(this->frame, cv::Point(det->p[0][0], det->p[0][1]),
                 cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 255, 0), 2);
        cv::line(this->frame, cv::Point(det->p[1][0], det->p[1][1]),
                 cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);
        cv::line(this->frame, cv::Point(det->p[2][0], det->p[2][1]),
                 cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 255, 0), 2);
        cv::line(this->frame, cv::Point(det->p[3][0], det->p[3][1]),
                 cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(0, 255, 0), 2);

        cv::imshow("charge", this->frame);
        cv::waitKey(1);
        //--------------------------------

        // gain the center point
        cv::Point2f center(det->c[0], det->c[1]);
        cv::circle(grayFrame, center, 5, cv::Scalar(0, 0, 255), -1);

        returnData.tag_id = det->id;
        // gain four corner point of tag
        std::vector<cv::Point2f> imagePoints;
        for (int j = 0; j < 4; j++)
        {
            imagePoints.push_back(cv::Point2f(det->p[j][0], det->p[j][1]));
        }

        // 定义标签的物理尺寸（单位：米）
        float tagSize = this->tagSize; // 例如，标签大小为0.165米

        // 定义标签在物理空间中的四个角点
        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(cv::Point3f(-tagSize / 2, tagSize / 2, 0));
        objectPoints.push_back(cv::Point3f(tagSize / 2, tagSize / 2, 0));
        objectPoints.push_back(cv::Point3f(tagSize / 2, -tagSize / 2, 0));
        objectPoints.push_back(cv::Point3f(-tagSize / 2, -tagSize / 2, 0));

        // 计算姿态估计
        cv::Mat rvec, tvec;
        cv::solvePnP(objectPoints, imagePoints, this->cameraMatrix, this->distCoeffs, rvec, tvec);

        // compute distance and angle
        double distance = norm(tvec);
        double angle = atan2(tvec.at<double>(0, 0), tvec.at<double>(2, 0)) * 180.0 / CV_PI;

        // std::cout << "Distance: " << distance << " meters" << std::endl;
        // std::cout << "Angle: " << angle << " degrees" << std::endl;
        // std::cout << "tag_id = " << returnData.tag_id << std::endl;

        memcpy(returnData.point, det->p, sizeof(double) * 8);
        returnData.distance = distance;
        returnData.angle = atan2(tvec.at<double>(0, 0), tvec.at<double>(2, 0));
    }
    apriltag_detections_destroy(detections);
    return returnData;
}