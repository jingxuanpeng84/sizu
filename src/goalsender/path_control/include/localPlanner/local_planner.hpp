#ifndef LOCAL_PLANNER__HPP
#define LOCAL_PLANNER__HPP
#include <vector>
#include <eigen3/Eigen/Dense>
namespace LOCAL_PLANNER
{
    enum DWARETURNTYPE
    {
        NOTARRIVALDEST,
        ARRIVALDEST,
    };
    struct plannerResult
    {
        double xV;
        double yV;
        double wV;
        LOCAL_PLANNER::DWARETURNTYPE dwaReturnType;
    };

    struct pathNode
    {
        double x;
        double y;
        double yaw;
    };
};

class localPlannerBase
{
public:
    /* data */
    LOCAL_PLANNER::pathNode destinationPoint;

public:
    localPlannerBase(LOCAL_PLANNER::pathNode destinationPoint_);
    ~localPlannerBase();
    void setDestinationPoint(LOCAL_PLANNER::pathNode destinationPoint_);
    virtual LOCAL_PLANNER::plannerResult plan(double x, double y, double yaw, double vX, double wV, std::vector<Eigen::Vector2d> &obstacle) = 0;
};

#endif