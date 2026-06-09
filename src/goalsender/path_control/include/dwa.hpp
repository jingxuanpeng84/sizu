//
// Created by guol on 2024/8/15.
//

#ifndef ROBOTICS_CPP_DWA_HPP
#define ROBOTICS_CPP_DWA_HPP

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include "path_control/include/localPlanner/local_planner.hpp"
using namespace std;
using namespace Eigen;

#define PI M_PI
#define LINE_NUM 181

namespace DWA_SPACE
{
    struct laddar_data
    {
        double x;
        double y;
        double yaw;
        double dist[LINE_NUM];
    };

    enum DWAWORKSTATE
    {
        NORMAL,
        ROTATE,
        EMERGENCYSTOP,
        AVOIDSEARCH,
    };

    
};

class DWA : public localPlannerBase
{
private:
    double dt;                         // 采样时间
    double v_min, v_max, w_min, w_max; // 线速度角速度边界
    double predict_time;               // 轨迹推算时间长度
    double a_vmax, a_wmax;             // 线加速度和角加速度最大值
    double v_sample, w_sample;         // 采样分辨率
    double alpha, beta, gamma;         // 轨迹评价函数系数
    double radius;                     // 用于判断是否到达目标点
    double obs_check_dr = 0.25;
    double judge_distance; // 若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
    double laddar_radius;  // 设定的雷达扫描半径，超出这个半径的距离认定为无物体

    
    // vector<Vector2d> obstacle;
    DWA_SPACE::DWAWORKSTATE dwaWorkState = DWA_SPACE::DWAWORKSTATE::NORMAL;
    Vector2d goal;

    unsigned int avoidTimes = 0;
    unsigned int avoidTimesCeil = 9;
    double avoidCheckRadius = 0.08;
    Vector2d checkPoint;
    double goalDirection = 0;
    unsigned int avoidSearchRotate = 0;

private:
    vector<double> calVelLimit();
    vector<double> calAccelLimit(double v, double w);
    vector<double> calObstacleLimit(VectorXd state, vector<Vector2d> obstacle);
    vector<double> calDynamicWindowVel(double v, double w, VectorXd state, vector<Vector2d> obstacle);
    double _dist(VectorXd state, vector<Vector2d> obstacle);
    vector<VectorXd> trajectoryPredict(VectorXd state, double v, double w);
    pair<vector<double>, vector<VectorXd>> trajectoryEvaluation(VectorXd state, Vector2d goal, vector<Vector2d> obstacle);

    double _heading(vector<VectorXd> trajectory, Vector2d goal);
    double _velocity(vector<VectorXd> trajectory);
    double _distance(vector<VectorXd> trajectory, vector<Vector2d> obstacle);

public:
    DWA(std::vector<LOCALPLANNER::pathNode> path_, unsigned long pathIndex_, double dt, double vMin, double vMax, double wMin, double wMax, double predictTime, double aVmax, double aWmax,
        double vSample, double wSample, double alpha, double beta, double gamma, double radius, double judgeDistance);

    VectorXd kinematicModel(VectorXd state, vector<double> control, double dt);

    double computerDAngle(VectorXd robotState, Vector2d goal); // return 0 <= abs(d_angle) <= PI

    // Vector2d computerObstacleCoordination(double dist, double angle, VectorXd robotState);

    // bool getObstacle(vector<Vector2d> &obstacle, DWA_SPACE::laddar_data laddarData);

    pair<vector<double>, vector<VectorXd>> dwaControl(VectorXd state, Vector2d goal, vector<Vector2d> obstacle);

    LOCALPLANNER::plannerResult plan(double x, double y, double yaw, vector<Vector2d> &obstacle) override;
    
};

#endif // ROBOTICS_CPP_DWA_HPP
