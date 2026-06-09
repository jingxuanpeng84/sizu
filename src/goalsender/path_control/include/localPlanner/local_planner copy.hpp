#ifndef LOCAL_PLANNER__HPP
#define LOCAL_PLANNER__HPP
#include <vector>
#include <eigen3/Eigen/Dense>
namespace LOCALPLANNER
{
    enum KEYPONITTYPE
    {
        NORMAL = 0,
        VISION = 1,
        CHARGE = 2,
    };
    struct plannerResult
    {
        double xV;
        double yV;
        double wV;
    };

    struct pathNode
    {
        double x;
        double y;
        double yaw;
        unsigned int pointType;
    };
};

class localPlannerBase
{
public:
    /* data */
    std::vector<LOCALPLANNER::pathNode> path;
    unsigned long pathIndex;

public:
    localPlannerBase(std::vector<LOCALPLANNER::pathNode> path_, unsigned long pathIndex_);
    ~localPlannerBase();

    virtual LOCALPLANNER::plannerResult plan(double x, double y, double yaw, std::vector<Eigen::Vector2d> &obstacle) = 0;
};



#endif