#include "path_control/include/localPlanner/local_planner.hpp"

localPlannerBase::localPlannerBase(LOCAL_PLANNER::pathNode destinationPoint_)
{
    this->destinationPoint = destinationPoint_;
}

localPlannerBase::~localPlannerBase()
{
}

void localPlannerBase::setDestinationPoint(LOCAL_PLANNER::pathNode destinationPoint_)
{
    this->destinationPoint = destinationPoint_;
}