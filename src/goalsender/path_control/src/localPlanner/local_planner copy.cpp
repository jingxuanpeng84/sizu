#include "path_control/include/localPlanner/local_planner.hpp"

localPlannerBase::localPlannerBase(std::vector<LOCALPLANNER::pathNode> path_, unsigned long pathIndex_)
{
    this->path = path_;
    this->pathIndex = pathIndex_;
}

localPlannerBase::~localPlannerBase()
{
}