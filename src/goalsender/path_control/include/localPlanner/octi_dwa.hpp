#ifndef OCTI_DWA_HPP
#define OCTI_DWA_HPP

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>
#include "path_control/include/localPlanner/local_planner.hpp"

namespace DWA_SPACE
{
    struct EvalutionStruct
    {
        double xV;
        double wV;
        double heading_eval;
        double dist_eval;
        double vel_eval;
        double yaw;
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
    double robot_radius;
    double obs_check_dr = 0.05;
    double judge_distance; // 若与障碍物的最小距离大于阈值（例如这里设置的阈值为robot_radius+0.2）,则设为一个较大的常值
    double laddar_radius;  // 设定的雷达扫描半径，超出这个半径的距离认定为无物体

private:
    std::vector<double> calVelLimit();
    std::vector<double> calAccelLimit(double v, double w);
    std::vector<double> calObstacleLimit(Eigen::VectorXd state, std::vector<Eigen::Vector2d> obstacle);
    std::vector<double> calDynamicWindowVel(double v, double w, Eigen::VectorXd state, std::vector<Eigen::Vector2d> obstacle);
    double _dist(Eigen::VectorXd state, std::vector<Eigen::Vector2d> obstacle);
    std::vector<Eigen::VectorXd> trajectoryPredict(Eigen::VectorXd state, double v, double w);
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> trajectoryEvaluation(Eigen::VectorXd state, Eigen::Vector2d goal, std::vector<Eigen::Vector2d> obstacle);

    double _heading(std::vector<Eigen::VectorXd> trajectory, Eigen::Vector2d goal);
    double _velocity(std::vector<Eigen::VectorXd> trajectory);
    double _distance(std::vector<Eigen::VectorXd> trajectory, std::vector<Eigen::Vector2d> obstacle);

public:
    DWA(LOCAL_PLANNER::pathNode destinationPoint_, double dt, double vMin, double vMax, double wMin, double wMax, double predictTime, double aVmax, double aWmax,
        double vSample, double wSample, double alpha, double beta, double gamma, double radius, double judgeDistance, double robot_radius);

    Eigen::VectorXd kinematicModel(Eigen::VectorXd state, std::vector<double> control, double dt);

    double computerDAngle(Eigen::VectorXd robotState, Eigen::Vector2d goal); // return 0 <= abs(d_angle) <= PI
    std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> dwaControl(Eigen::VectorXd state, Eigen::Vector2d goal, std::vector<Eigen::Vector2d> obstacle);

    LOCAL_PLANNER::plannerResult plan(double x, double y, double yaw, double vX, double wV, std::vector<Eigen::Vector2d> &obstacle) override;
};

#endif