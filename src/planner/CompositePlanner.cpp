#include "aikido/planner/CompositePlanner.hpp"

#include <algorithm>
#include <dart/common/Console.hpp>

namespace aikido {
namespace planner {

//==============================================================================
CompositePlanner::CompositePlanner() : mPlanners()
{
  // Do nothing
}

//==============================================================================
CompositePlanner::CompositePlanner(const std::vector<PlannerPtr>& planners)
  : mPlanners(planners)
{
  // Do nothing
}

//==============================================================================
void CompositePlanner::addPlanner(PlannerPtr planner)
{
  if (std::find(mPlanners.begin(), mPlanners.end(), planner) != mPlanners.end())
  {
    dtwarn << "[CompositePlanner::addPlanner] Attempts to add a planner that "
              "already exists in this CompositePlanner.\n";
    return;
  }

  mPlanners.emplace_back(std::move(planner));
}

//==============================================================================
bool CompositePlanner::hasPlanner(const Planner* planner)
{
  if (std::find_if(
          mPlanners.begin(),
          mPlanners.end(),
          [&planner](const PlannerPtr& val) -> bool {
            return (val.get() == planner);
          })
      != mPlanners.end())
  {
    return true;
  }

  return false;
}

//==============================================================================
const std::vector<PlannerPtr>& CompositePlanner::getPlanners() const
{
  return mPlanners;
}

//==============================================================================
PlannerPtr CompositePlanner::getPlanner(std::size_t index)
{
  if (index > mPlanners.size())
    throw std::invalid_argument("index is out of bound");

  return mPlanners[index];
}

//==============================================================================
bool CompositePlanner::canPlan(const Problem* problem) const
{
  for (const auto& planner : mPlanners)
  {
    if (planner->canPlan(problem))
      return true;
  }

  return false;
}

} // namespace planner
} // namespace aikido
