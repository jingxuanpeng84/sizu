#include "path_control/include/dwa.hpp"
using namespace std;
using namespace Eigen;
DWA::DWA(std::vector<LOCALPLANNER::pathNode> path_, unsigned long pathIndex_, double dt, double vMin, double vMax, double wMin, double wMax,
         double predictTime, double aVmax, double aWmax, double vSample,
         double wSample, double alpha, double beta, double gamma, double radius,
         double judgeDistance)
    : dt(dt), v_min(vMin), v_max(vMax), w_min(wMin), w_max(wMax),
      predict_time(predictTime), a_vmax(aVmax), a_wmax(aWmax),
      v_sample(vSample), w_sample(wSample), alpha(alpha), beta(beta),
      gamma(gamma), radius(radius), judge_distance(judgeDistance), localPlannerBase(path_, pathIndex_) {}

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
vector<double> DWA::calAccelLimit(double v, double w)
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
vector<double> DWA::calObstacleLimit(VectorXd state,
                                     vector<Vector2d> obstacle)
{
  double v_low = v_min;
  double v_high = sqrt(2 * _dist(state, obstacle) * a_vmax);
  double w_low = w_min;
  double w_high = sqrt(2 * _dist(state, obstacle) * a_wmax);
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
vector<double> DWA::calDynamicWindowVel(double v, double w, VectorXd state,
                                        vector<Vector2d> obstacle)
{

  vector<double> Vm = calVelLimit();
  vector<double> Vd = calAccelLimit(v, w);
  vector<double> Va = calObstacleLimit(state, obstacle);
  double a = max({Vm[0], Vd[0], Va[0]});
  double b = min({Vm[1], Vd[1], Va[1]});
  double c = max({Vm[2], Vd[2], Va[2]});
  double d = min({Vm[3], Vd[3], Va[3]});
  // std::cout << "window vmax = " << a << " vmin= " << b << " wmax = " << c << " wmin = " << d << std::endl;
  return {a, b, c, d};
}

/**
 * 计算当前移动机器人距离障碍物最近的几何距离
 * @param state 当前机器人状态
 * @param obstacle 所有障碍物位置
 * @return 移动机器人距离障碍物最近的几何距离
 */
double DWA::_dist(VectorXd state, vector<Vector2d> obstacle)
{
  double min_dist = 100000;
  for (Vector2d obs : obstacle)
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
VectorXd DWA::kinematicModel(VectorXd state, vector<double> control, double dt)
{
  state(0) += control[0] * cos(state(2)) * dt;
  state(1) += control[0] * sin(state(2)) * dt;
  state(2) += control[1] * dt; // update yaw
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
vector<VectorXd> DWA::trajectoryPredict(VectorXd state, double v, double w)
{
  vector<VectorXd> trajectory;
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
pair<vector<double>, vector<VectorXd>>
DWA::trajectoryEvaluation(VectorXd state, Vector2d goal,
                          vector<Vector2d> obstacle)
{
  double G_max = -10000000;        // 最优评价
  vector<VectorXd> trajectory_opt; // 最优轨迹
  trajectory_opt.push_back(state);
  vector<double> control_opt = {0., 0.}; // 最优控制
  vector<double> dynamic_window_vel = calDynamicWindowVel(
      state(3), state(4), state, obstacle); // 第1步--计算速度空间

  double sum_heading = 0.0, sum_dist = 0.0,
         sum_vel = 0.0; // 统计全部采样轨迹的各个评价之和，便于评价的归一化
  double v = dynamic_window_vel[0];
  double w = dynamic_window_vel[2];
  // while (v < dynamic_window_vel[1])
  // {
  //   while (w < dynamic_window_vel[3])
  //   {
  //     vector<VectorXd> trajectory = trajectoryPredict(state, v, w);
  //     double heading_eval = alpha * _heading(trajectory, goal);
  //     double dist_eval = beta * _distance(trajectory, obstacle);
  //     double vel_eval = gamma * _velocity(trajectory);
  //     sum_vel += vel_eval;
  //     sum_dist += dist_eval;
  //     sum_heading += heading_eval;
  //     w += w_sample;
  //   }
  //   v += v_sample;
  //   w = dynamic_window_vel[2];
  // }
  sum_heading = 1.0, sum_dist = 1.0, sum_vel = 1.0; // 不进行归一化
  v = dynamic_window_vel[0];
  w = dynamic_window_vel[2];
  while (v < dynamic_window_vel[1])
  {
    while (w < dynamic_window_vel[3])
    {
      vector<VectorXd> trajectory =
          trajectoryPredict(state, v, w); // 第2步--轨迹推算

      double heading_eval = alpha * _heading(trajectory, goal) / sum_heading;
      double dist_eval = beta * _distance(trajectory, obstacle) / sum_dist;
      double vel_eval = gamma * _velocity(trajectory) / sum_vel;
      double G = heading_eval + dist_eval + vel_eval; // 第3步--轨迹评价

      if (G_max <= G)
      {
        G_max = G;
        trajectory_opt = trajectory;
        control_opt = {v, w};
      }
      w += w_sample;
    }
    v += v_sample;
    w = dynamic_window_vel[2];
  }
  pair<vector<double>, vector<VectorXd>> result;
  result.first = control_opt;
  result.second = trajectory_opt;
  return result;
}

/**
 * 方位角评价函数
 * 评估在当前采样速度下产生的轨迹终点位置方向与目标点连线的夹角的误差
 * @param trajectory 轨迹，dim:[n,5]
 * @param goal 目标点位置[x,y]
 * @return 方位角评价数值
 */
double DWA::_heading(vector<VectorXd> trajectory, Vector2d goal)
{
  // Vector2d dxy = goal - trajectory[trajectory.size() - 1].head(2);
  // double error_angle = atan2(dxy(1), dxy(0));
  // double cost_angle = error_angle - trajectory[trajectory.size() - 1](2);
  // double cost = PI - abs(cost_angle);

  Vector2d dxy = goal - trajectory[trajectory.size() - 1].head(2);
  double error_angle = atan2(dxy(1), dxy(0));
  double cost_angle = error_angle - trajectory[trajectory.size() - 1](2);
  if (cost_angle > M_PI)
  {
    cost_angle = cost_angle - 2 * M_PI;
  }
  else if (cost_angle < -M_PI)
  {
    cost_angle = cost_angle + 2 * M_PI;
  }
  double cost = M_PI - abs(cost_angle);
  // std::cout << " errangle = " << error_angle << " costangle = " << cost_angle << std::endl;
  return cost;
}

/**
 * 速度评价函数
 * 表示当前的速度大小，可以用模拟轨迹末端位置的线速度的大小来表示
 * @param trajectory 轨迹，dim:[n,5]
 * @return 速度评价值
 */
double DWA::_velocity(vector<VectorXd> trajectory)
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
double DWA::_distance(vector<VectorXd> trajectory, vector<Vector2d> obstacle)
{
  double min_r = 10000000;
  for (Vector2d obs : obstacle)
  {
    for (VectorXd state : trajectory)
    {
      Vector2d dxy = obs - state.head(2);
      double r = dxy.norm();
      min_r = min_r > r ? r : min_r;
    }
  }
  if (min_r < radius + this->obs_check_dr)
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
pair<vector<double>, vector<VectorXd>>
DWA::dwaControl(VectorXd state, Vector2d goal, vector<Vector2d> obstacle)
{
  pair<vector<double>, vector<VectorXd>> res = trajectoryEvaluation(state, goal, obstacle);
  return res;
}

/**
 * 将雷达障碍物数据计算出障碍物的坐标
 * @param dist 障碍物距离robot的距离
 * @param angle 障碍物与robot正向的夹角
 * @param state 机器人当前状态--[x,y,yaw,v,w]
 * @return  障碍物的坐标
 */
// Vector2d DWA::computerObstacleCoordination(double dist, double angle, VectorXd robotState)
// {
//   Vector2d coordination;
//   // std::cout << "compute enter\n";
//   // robot yaw > 0 or yaw < 0, the compute is the same
//   coordination[0] = robotState[0] + dist * cos(angle - PI / 2 - robotState[2]);
//   coordination[1] = (robotState[1] + dist * sin(-(angle - PI / 2 - robotState[2])));
//   // std::cout << "compute success\n";
//   return coordination;
// }

// bool DWA::getObstacle(vector<Vector2d> &obstacle, DWA_SPACE::laddar_data laddarData)
// {
//   obstacle.clear();
//   VectorXd robotState(5);
//   robotState[0] = laddarData.x;
//   robotState[1] = laddarData.y;
//   robotState[2] = laddarData.yaw;
//   // std::cout << "gto en\n";
//   for (unsigned long i = 0; i <= LINE_NUM; ++i)
//   {
//     if (laddarData.dist[i] <= this->laddar_check_radius_max && laddarData.dist[i] >= this->laddar_check_radius_min)
//     {
//       obstacle.push_back(this->computerObstacleCoordination(laddarData.dist[i], i * this->d_laddar_angle, robotState));
//     }
//   }
//   return true;
// }

double DWA::computerDAngle(VectorXd robotState, Vector2d goal)
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

LOCALPLANNER::plannerResult DWA::plan(double x, double y, double yaw, vector<Vector2d> &obstacle)
{
  VectorXd state(5); //[x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
  LOCALPLANNER::plannerResult res;
  // init robot state
  this->goal[0] = this->path[this->pathIndex].x;
  this->goal[1] = this->path[this->pathIndex].y;

  state[0] = x;
  state[1] = y;
  state[2] = yaw;

  // #begin -> judge arrive at goal
  double dist_to_goal = (state.head(2) - goal).norm();

  if (dist_to_goal <= this->radius && this->dwaWorkState == DWA_SPACE::DWAWORKSTATE::NORMAL)
  {
    this->dwaWorkState = DWA_SPACE::DWAWORKSTATE::ROTATE;
    std::cout << "arrive point : x = " << goal[0] << " y = " << goal[1] << std::endl;
    this->pathIndex = (this->pathIndex + 1) % this->path.size();

    // #begin -> update goal
    this->goal[0] = this->path[this->pathIndex].x;
    this->goal[1] = this->path[this->pathIndex].y;
    // #end -> update goal
    res.xV = 0;
    res.yV = 0;
    res.wV = 0;
    return res;
  }
  else if (dist_to_goal > radius && dist_to_goal <= radius + 10 && abs(this->computerDAngle(state, goal)) >= M_PI * 2 / 4 && this->dwaWorkState == DWA_SPACE::DWAWORKSTATE::NORMAL)
  {
    this->dwaWorkState = DWA_SPACE::DWAWORKSTATE::ROTATE;
    std::cout << "obs arrived goal\n";
    std::cout << "arrive point : x = " << goal[0] << " y = " << goal[1] << std::endl;
    this->pathIndex = (this->pathIndex + 1) % this->path.size();

    // #begin -> update goal
    this->goal[0] = this->path[this->pathIndex].x;
    this->goal[1] = this->path[this->pathIndex].y;
    // #end -> update goal
    res.xV = 0;
    res.yV = 0;
    res.wV = 0;
    return res;
  }
  if (this->dwaWorkState == DWA_SPACE::DWAWORKSTATE::NORMAL)
  {
    // #begin -> dwa controler
    if ((this->checkPoint - state.head(2)).norm() < this->avoidCheckRadius)
    {
      this->avoidTimes++;
      std::cout << "times = " << this->avoidTimes << std::endl;
    }
    else
    {
      this->avoidTimes = 0;
    }
    this->checkPoint = state.head(2);
    if (this->avoidTimes > this->avoidTimesCeil)
    {
      this->avoidTimes = 0;
      this->dwaWorkState = DWA_SPACE::DWAWORKSTATE::AVOIDSEARCH;
    }
    pair<vector<double>, vector<VectorXd>> dwares = this->dwaControl(state, goal, obstacle);
    res.xV = dwares.first[0];
    res.yV = 0;
    res.wV = dwares.first[1];
    return res;
    // #end -> dwa controler
  }
  else if (this->dwaWorkState == DWA_SPACE::DWAWORKSTATE::ROTATE)
  {
    // #begin -> turn around manipulation
    double err_angle = this->computerDAngle(state, this->goal);

    // #begin -> rotate
    if (abs(err_angle) > 5 * M_PI / 180)
    {
      res.xV = 0;
      res.yV = 0;
      res.wV = (err_angle) / abs(err_angle) * 0.5;
      return res;
      // #end -> rotate
      // #end -> turn around manipulation
    }
    else
    {
      this->dwaWorkState = DWA_SPACE::DWAWORKSTATE::NORMAL;
      res.xV = 0;
      res.yV = 0;
      res.wV = 0;
      return res;
    }
  }
  else if (this->dwaWorkState == DWA_SPACE::DWAWORKSTATE::EMERGENCYSTOP)
  {
    res.xV = 0;
    res.yV = 0;
    res.wV = 0;
    return res;
  }
  else if (this->dwaWorkState == DWA_SPACE::DWAWORKSTATE::AVOIDSEARCH)
  {
    std::vector<std::pair<double, double>> computerVector; // first: deyaAngle, second: dist
    std::vector<double> detaAngleVector;
    if (this->avoidSearchRotate == 0)
    {
      // computerVector.push_back(std::pair<double, double>(M_PI / 2, 2)); // push left 0
      for (auto obs : obstacle)
      {
        std::pair<double, double> temp;
        temp.first = this->computerDAngle(state, obs);
        temp.second = (obs - state.head(2)).norm();
        computerVector.push_back(temp);
        // std::cout << "Dangle = " << temp.first << " dist = " << temp.second << std::endl;
      }
      // computerVector.push_back(std::pair<double, double>(-M_PI / 2, 2)); // push right 180
      std::pair<double, double> preNode = computerVector[0];
      std::cout << "computerVector.size() = " << computerVector.size() << std::endl;
      for (unsigned int i = 1; i < computerVector.size(); i++)
      {
        if (abs(preNode.second - computerVector[i].second) >= 1.4)
        {
          std::cout << "continue : " << preNode.first << " left = " << computerVector[i].first << std::endl;
          continue;
        }
        if (preNode.first - computerVector[i].first > 2 * M_PI / 180)
        {
          std::cout << "Da = " << preNode.first - computerVector[i].first << std::endl;
          if (preNode.second > computerVector[i].second)
          {
            double detaDist = (preNode.first - computerVector[i].first) * (preNode.second + computerVector[i].second) / 2;
            std::cout << "dist = " << detaDist << std::endl;
            if (detaDist >= 0.4) // 0.45 dog can access len
            {
              std::cout << "---------da = " << (preNode.first + computerVector[i].first) / 2 << std::endl;
              std::cout << "--- pre a = " << preNode.first << " after a = " << computerVector[i].first << std::endl;
              detaAngleVector.push_back((preNode.first + computerVector[i].first) / 2);
            }
          }
          else
          {
            double detaDist = (preNode.first - computerVector[i].first) * (preNode.second + computerVector[i].second) / 2;
            if (detaDist >= 0.4) // 0.45 dog can access len
            {
              std::cout << "---------da = " << (preNode.first + computerVector[i].first) / 2 << std::endl;
              std::cout << "--- pre a = " << preNode.first << " after a = " << computerVector[i].first << std::endl;
              detaAngleVector.push_back((preNode.first + computerVector[i].first) / 2);
            }
          }
        }
        preNode = computerVector[i];
      }
      if (detaAngleVector.empty())
      {
        this->dwaWorkState = DWA_SPACE::DWAWORKSTATE::NORMAL;
        std::cout << "Can not avoid ! EMERGENCYSTOP\n";
        res.xV = 0;
        res.yV = 0;
        res.wV = 0;
        return res;
      }
      // find closer detaAngle
      double absMinDetaAngle = M_PI;
      double minDetaAngle = 0;
      for (auto detaAngleItem : detaAngleVector)
      {
        if (abs(detaAngleItem) < absMinDetaAngle)
        {
          absMinDetaAngle = abs(detaAngleItem);
          minDetaAngle = detaAngleItem;
        }
      }
      std::cout << "detaanglevec size = " << detaAngleVector.size() << std::endl;
      if (state[2] + minDetaAngle > M_PI)
      {
        this->goalDirection = state[2] + minDetaAngle - 2 * M_PI;
      }
      else if (state[2] + minDetaAngle < -M_PI)
      {
        this->goalDirection = state[2] + minDetaAngle + 2 * M_PI;
      }
      else
      {
        this->goalDirection = state[2] + minDetaAngle;
      }
      this->avoidSearchRotate = 1;
      std::cout << "AVOIDSEARCH : deta angle = " << minDetaAngle << std::endl;
    }
    double errAngle = this->goalDirection - state[2];
    if (errAngle > M_PI)
    {
      errAngle = errAngle - 2 * M_PI;
    }
    else if (errAngle < -M_PI)
    {
      errAngle = errAngle + 2 * M_PI;
    }
    if (abs(errAngle) > 4 * M_PI / 180 && this->avoidSearchRotate == 1)
    {
      // std::cout << "abs(this->goalDirection - state[2]) = " << abs(this->goalDirection - state[2]) << std::endl;
      // std::cout << "state[2] = " << state[2] << " gD = " << this->goalDirection << std::endl;
      res.xV = 0;
      res.yV = 0;
      res.wV = abs(errAngle) / (errAngle) * 0.4;
    }
    else
    {
      this->dwaWorkState = DWA_SPACE::DWAWORKSTATE::NORMAL;
      this->avoidSearchRotate = 0;
      res.xV = 0;
      res.yV = 0;
      res.wV = 0;
    }
    return res;
  }
  return res;
}
// #end -> judge arrive at goal
