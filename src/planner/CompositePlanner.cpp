#include "aikido/planner/CompositePlanner.hpp"

#include <algorithm>
#include <dart/common/Console.hpp>

namespace aikido {
namespace planner {

//==============================================================================
CompositePlanner::CompositePlanner(
    statespace::ConstStateSpacePtr stateSpace,
    const std::vector<PlannerPtr>& planners)
  : Planner(std::move(stateSpace)), mPlanners(planners)
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
bool CompositePlanner::hasPlanner(const Planner* planner) const
{
  return std::find_if(
          mPlanners.begin(),
          mPlanners.end(),
          [&planner](const PlannerPtr& val) -> bool {
            return (val.get() == planner);
          })
      != mPlanners.end();
}

//==============================================================================
const std::vector<PlannerPtr>& CompositePlanner::getPlanners()
{
  return mPlanners;
}

//==============================================================================
std::size_t CompositePlanner::getNumPlanners() const
{
  return mPlanners.size();
}

//==============================================================================
PlannerPtr CompositePlanner::getPlanner(std::size_t index)
{
  if (index > mPlanners.size())
    throw std::invalid_argument("index is out of bound");

  return mPlanners[index];
}

//==============================================================================
ConstPlannerPtr CompositePlanner::getPlanner(std::size_t index) const
{
  return const_cast<CompositePlanner*>(this)->getPlanner(index);
}

//==============================================================================
bool CompositePlanner::canSolve(const Problem* problem) const
{
  for (const auto& planner : mPlanners)
  {
    if (planner->canSolve(problem))
      return true;
  }

  return false;
}

} // namespace planner
} // namespace aikido
