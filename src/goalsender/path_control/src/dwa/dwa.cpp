#include "path_control/include/localPlanner/octi_dwa.hpp"
// using namespace std;
// using namespace Eigen;
DWA::DWA(LOCAL_PLANNER::pathNode destinationPoint_, double dt, double vMin, double vMax, double wMin, double wMax,
         double predictTime, double aVmax, double aWmax, double vSample,
         double wSample, double alpha, double beta, double gamma, double radius,
         double judgeDistance, double robot_radius)
    : dt(dt), v_min(vMin), v_max(vMax), w_min(wMin), w_max(wMax),
      predict_time(predictTime), a_vmax(aVmax), a_wmax(aWmax),
      v_sample(vSample), w_sample(wSample), alpha(alpha), beta(beta),
      gamma(gamma), radius(radius), judge_distance(judgeDistance), robot_radius(robot_radius), localPlannerBase(destinationPoint_) {}

/**
 * 计算速度边界限制Vm
 * @return 速度边界限制后的速度空间Vm
 */
std::vector<double> DWA::calVelLimit()
{
  return {this->v_min, this->v_max, this->w_min, this->w_max};
}

/**
 * 计算加速度限制Vd
 * @param v 当前机器人状态
 * @param w 障碍物位置
 * @return
 */
std::vector<double> DWA::calAccelLimit(double v, double w)
{
  double v_low = v - a_vmax * dt;
  double v_high = v + a_vmax * dt;
  double w_low = w - a_wmax * dt;
  double w_high = w + a_wmax * dt;
  return {v_low, v_high, w_low, w_high};
}

/**
 * 环境障碍物限制Va
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return 移动机器人不与周围障碍物发生碰撞的速度空间Va
 */
std::vector<double> DWA::calObstacleLimit(Eigen::VectorXd state,
                                          std::vector<Eigen::Vector2d> obstacle)
{
  // double v_low = v_min;
  // double v_high = sqrt(2 * _dist(state, obstacle) * a_vmax);
  // double w_low = std::max(sqrt(2 * _dist(state, obstacle) * w_min), w_min); //w_min;
  // double w_high = sqrt(2 * _dist(state, obstacle) * a_wmax);
  double v_low = v_min;
  double v_high = state[3] + this->a_vmax * this->dt;
  v_high = std::min(v_high, this->v_max);
  double w_low = state[4] - this->a_wmax * this->dt;
  w_low = std::max(w_low, this->w_min); 
  double w_high =state[4] + this->a_wmax * this->dt; 
  w_high = std::min(w_high, this->w_max); 
  return {v_low, v_high, w_low, w_high};
}

/**
 * 速度采样,得到速度空间窗口
 * @param v 当前时刻线速度
 * @param w 当前时刻角速度
 * @param state 当前机器人状态
 * @param obstacle 障碍物位置
 * @return [v_low,v_high,w_low,w_high]: 最终采样后的速度空间
 */
std::vector<double> DWA::calDynamicWindowVel(double v, double w, Eigen::VectorXd state,
                                             std::vector<Eigen::Vector2d> obstacle)
{

  // std::vector<double> Vm = calVelLimit();
  // std::vector<double> Vd = calAccelLimit(v, w);
  // std::vector<double> Va = calObstacleLimit(state, obstacle);
  // double a = std::max({Vm[0], Vd[0], Va[0]});
  // double b = std::min({Vm[1], Vd[1], Va[1]});
  // double c = std::max({Vm[2], Vd[2], Va[2]});
  // double d = std::min({Vm[3], Vd[3], Va[3]});
  // std::cout << "window vmax = " << a << " vmin= " << b << " wmax = " << c << " wmin = " << d << std::endl;
  // return {a, b, c, d};
  double v_low = v_min;
  double v_high = v + this->a_vmax * this->dt;
  v_high = std::min(v_high, this->v_max);
  double w_low = w - this->a_wmax * this->dt;
  w_low = std::max(w_low, this->w_min); 
  double w_high = w + this->a_wmax * this->dt; 
  w_high = std::min(w_high, this->w_max); 
  return {v_low, v_high, w_low, w_high};
}

/**
 * 计算当前移动机器人距离障碍物最近的几何距离
 * @param state 当前机器人状态
 * @param obstacle 所有障碍物位置
 * @return 移动机器人距离障碍物最近的几何距离
 */
double DWA::_dist(Eigen::VectorXd state, std::vector<Eigen::Vector2d> obstacle)
{
  double min_dist = 100000;
  for (Eigen::Vector2d obs : obstacle)
  {
    double distance = (obs - state.head(2)).norm(); // 向量求距离
    min_dist = distance > min_dist ? min_dist : distance;
  }
  return min_dist;
}

/**
 * 机器人运动学模型
 * @param state 状态量---x,y,yaw,v,w
 * @param control 控制量---v,w,线速度和角速度
 * @param dt 采样时间
 * @return 下一步的状态
 */
Eigen::VectorXd DWA::kinematicModel(Eigen::VectorXd state, std::vector<double> control, double dt)
{
  state(0) += control[0] * cos(state(2)) * dt;
  state(1) += control[0] * sin(state(2)) * dt;
  if(state(2) + control[1] * dt > M_PI)
  {
    state(2) = state(2) + control[1] * dt - 2 * M_PI;
  }
  else if(state(2) + control[1] * dt < -M_PI)
  {
    state(2) = 2 * M_PI + state(2) + control[1] * dt;
  }
  else
  {
    state(2) += control[1] * dt;
  }
  // state(2) += control[1] * dt; // update yaw
  state(3) = control[0];
  state(4) = control[1];

  return state;
}

/**
 * 轨迹推算，根据当前速度对预测出轨迹
 * @param state 当前状态---x,y,yaw,v,w
 * @param v 当前时刻线速度
 * @param w 当前时刻线速度
 * @return 推算后的轨迹
 */
std::vector<Eigen::VectorXd> DWA::trajectoryPredict(Eigen::VectorXd state, double v, double w)
{
  std::vector<Eigen::VectorXd> trajectory;
  trajectory.push_back(state);
  double time = 0;
  while (time <= predict_time)
  {
    state = kinematicModel(state, {v, w}, dt);
    trajectory.push_back(state);
    time += dt;
  }
  return trajectory;
}

/**
 * 轨迹评价函数,评价越高，轨迹越优
 * @param state 当前状态---x,y,yaw,v,w
 * @param goal 目标点位置，[x,y]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 最优控制量、最优轨迹
 */
std::pair<std::vector<double>, std::vector<Eigen::VectorXd>>
DWA::trajectoryEvaluation(Eigen::VectorXd state, Eigen::Vector2d goal,
                          std::vector<Eigen::Vector2d> obstacle)
{
  // std::cout << "Input v = " << state(3)<< " " << state(4) << std::endl;
  double G_max = -10000000;                    // 最优评价
  std::vector<Eigen::VectorXd> trajectory_opt; // 最优轨迹
  trajectory_opt.push_back(state);
  std::vector<double> control_opt = {0., 0.}; // 最优控制
  std::vector<double> dynamic_window_vel = calDynamicWindowVel(
      state(3), state(4), state, obstacle); // 第1步--计算速度空间

  // std::cout << "window = " << dynamic_window_vel[0] << " " << dynamic_window_vel[1] << " " << dynamic_window_vel[2] << " " << dynamic_window_vel[3] << std::endl;
  double sum_heading = 0.0, sum_dist = 0.0,
         sum_vel = 0.0; // 统计全部采样轨迹的各个评价之和，便于评价的归一化
  double v = dynamic_window_vel[0];
  double w = dynamic_window_vel[2];
  std::vector<DWA_SPACE::EvalutionStruct> evalutionVector;
  while (v < dynamic_window_vel[1])
  {
    while (w < dynamic_window_vel[3])
    {
      std::vector<Eigen::VectorXd> trajectory = trajectoryPredict(state, v, w);
      double heading_eval = _heading(trajectory, goal);
      double dist_eval = _distance(trajectory, obstacle);
      double vel_eval = _velocity(trajectory);
      sum_vel += vel_eval;
      sum_dist += dist_eval;
      sum_heading += heading_eval;
      // std::cout << " last " << trajectory.back()[0] << " " << trajectory.back()[1] <<std::endl;
      // std::cout << "eva scoal = " << heading_eval << " " << dist_eval << " " << vel_eval << std::endl;
      DWA_SPACE::EvalutionStruct tempEvaluVector = {v, w, heading_eval, dist_eval, vel_eval, trajectory[trajectory.size() - 1](2)};
      evalutionVector.emplace_back(tempEvaluVector);
      w += w_sample;
    }
    v += v_sample;
    w = dynamic_window_vel[2];
  }
  for (auto tempEvalution : evalutionVector)
  {
    double heading_eval = alpha * tempEvalution.heading_eval / sum_heading;
    double dist_eval = beta * tempEvalution.dist_eval / sum_dist;
    double vel_eval = gamma * tempEvalution.vel_eval / sum_vel;
    double G = heading_eval + dist_eval + vel_eval; // 第3步--轨迹评价
    // std::cout << "Eval\n";
    // std::cout << "eva scoal = " << heading_eval << " " << dist_eval << " " << vel_eval << std::endl;
    if (G_max <= G)
    {
      G_max = G;
      // trajectory_opt = trajectory;
      // std::cout << "eva v = " << tempEvalution.xV << " " << tempEvalution.wV << " yaw = " << tempEvalution.yaw << std::endl;
      
      control_opt = {tempEvalution.xV, tempEvalution.wV};
    }
  }
  // while (v < dynamic_window_vel[1])
  // {
  //   while (w < dynamic_window_vel[3])
  //   {
  //     DWA_SPACE::EvalutionStruct tempEvalution = evalutionVector.back();
  //     double heading_eval = alpha * tempEvalution.heading_eval / sum_heading;
  //     double dist_eval = beta * tempEvalution.dist_eval / sum_dist;
  //     double vel_eval = gamma * tempEvalution.vel_eval / sum_vel;
  //     double G = heading_eval + dist_eval + vel_eval; // 第3步--轨迹评价

  //     if (G_max <= G)
  //     {
  //       G_max = G;
  //       // trajectory_opt = trajectory;
  //       control_opt = {tempEvalution.xV, tempEvalution.wV};
  //     }
  //     w += w_sample;
  //   }
  //   v += v_sample;
  //   w = dynamic_window_vel[2];
  // }
  std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> result;
  result.first = control_opt;
  // result.second = trajectory_opt;
  return result;
}

/**
 * 方位角评价函数
 * 评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差
 * @param trajectory 轨迹，dim:[n,5]
 * @param goal 目标点位置[x,y]
 * @return 方位角评价数值
 */
double DWA::_heading(std::vector<Eigen::VectorXd> trajectory, Eigen::Vector2d goal)
{
  // Eigen::Vector2d dxy = goal - trajectory[trajectory.size() - 1].head(2);
  // double error_angle = atan2(dxy(1), dxy(0));
  // double cost_angle = error_angle - trajectory[trajectory.size() - 1](2);
  // double cost = M_PI - abs(cost_angle);

  Eigen::Vector2d dxy = goal - trajectory[trajectory.size() - 1].head(2);
  double error_angle = atan2(dxy(1), dxy(0));
  // double angleModify = (trajectory[trajectory.size() - 1](2) / 2 * M_PI) == 0 ? std::fmod(trajectory[trajectory.size() - 1](2), M_PI) : -std::fmod(trajectory[trajectory.size() - 1](2), M_PI);
  double cost_angle = error_angle - trajectory[trajectory.size() - 1](2);
  if (cost_angle > M_PI)
  {
    cost_angle = cost_angle - 2 * M_PI; 
  }
  else if (cost_angle < -M_PI)
  {
    cost_angle = cost_angle + 2 * M_PI; 
  }
  // double cost = M_PI - abs(cost_angle);
  // std::cout << "angleModify = " << angleModify <<std::endl;
  double cost = cos(cost_angle) * 1;
  // std::cout << " errangle = " << error_angle << " costangle = " << cost_angle << std::endl;
  // std::cout << " errangle = " << error_angle << " cost = " << cost << " yaw = " << trajectory[trajectory.size() - 1](2) <<std::endl;
  return cost;
}

/**
 * 速度评价函数
 * 表示当前的速度大小，可以用模拟轨迹末端位置的线速度的大小来表示
 * @param trajectory 轨迹，dim:[n,5]
 * @return 速度评价值
 */
double DWA::_velocity(std::vector<Eigen::VectorXd> trajectory)
{
  return trajectory[trajectory.size() - 1](3);
}

/**
 * 距离评价函数
 * 表示当前速度下对应模拟轨迹与障碍物之间的最近距离；
 * 如果没有障碍物或者最近距离大于设定的阈值，那么就将其值设为一个较大的常数值。
 * @param trajectory 轨迹，dim:[n,5]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return 距离评价值
 */
double DWA::_distance(std::vector<Eigen::VectorXd> trajectory, std::vector<Eigen::Vector2d> obstacle)
{
  double min_r = 10000000;
  for (Eigen::Vector2d obs : obstacle)
  {
    for (Eigen::VectorXd state : trajectory)
    {
      Eigen::Vector2d dxy = obs - state.head(2);
      double r = dxy.norm() - this->obs_check_dr;
      min_r = min_r > r ? r : min_r;
    }
  }
  if (min_r < this->robot_radius) //+ this->obs_check_dr
  {
    return min_r;
  }
  else
  {
    return judge_distance;
  }
}

/**
 * 滚动窗口算法控制器
 * @param state 机器人当前状态--[x,y,yaw,v,w]
 * @param goal 目标点位置，[x,y]
 * @param obstacle 障碍物位置，dim:[num_ob,2]
 * @return  最优控制量[v,w]、最优轨迹
 */
std::pair<std::vector<double>, std::vector<Eigen::VectorXd>>
DWA::dwaControl(Eigen::VectorXd state, Eigen::Vector2d goal, std::vector<Eigen::Vector2d> obstacle)
{
  std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> res = trajectoryEvaluation(state, goal, obstacle);
  return res;
}


double DWA::computerDAngle(Eigen::VectorXd robotState, Eigen::Vector2d goal)
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

LOCAL_PLANNER::plannerResult DWA::plan(double x, double y, double yaw, double vX, double wV, std::vector<Eigen::Vector2d> &obstacle)
{
  Eigen::VectorXd state(5); //[x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
  LOCAL_PLANNER::plannerResult res;
  // init robot state
  Eigen::Vector2d goal;
  goal[0] = this->destinationPoint.x;
  goal[1] = this->destinationPoint.y;

  state[0] = x;
  state[1] = y;
  state[2] = yaw;
  state[3] = vX;
  state[4] = wV;

  // #begin -> judge arrive at goal
  double dist_to_goal = (state.head(2) - goal).norm();
  if (dist_to_goal <= this->radius)
  {
    res.xV = 0;
    res.yV = 0;
    res.wV = 0;
    res.dwaReturnType = LOCAL_PLANNER::DWARETURNTYPE::ARRIVALDEST;
    return res;
  }
  // #begin -> dwa controler
  std::pair<std::vector<double>, std::vector<Eigen::VectorXd>> dwaRes = this->dwaControl(state, goal, obstacle);
  res.xV = dwaRes.first[0];
  res.yV = 0;
  res.wV = dwaRes.first[1];
  // std::cout << "vYaw = " << res.wV << std::endl;
  res.dwaReturnType = LOCAL_PLANNER::DWARETURNTYPE::NOTARRIVALDEST;
  return res;
  // #end -> dwa controler
}
// #end -> judge arrive at goal
