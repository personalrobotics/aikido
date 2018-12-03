#include "aikido/planner/CompositePlanner.hpp"

#include <algorithm>
#include <utility>

namespace aikido {
namespace planner {

//==============================================================================
CompositePlanner::CompositePlanner(
    statespace::ConstStateSpacePtr stateSpace,
    const std::vector<PlannerPtr>& planners)
  : Planner(std::move(stateSpace)), mPlanners(planners)
{
  for (auto planner : mPlanners)
  {
    if (!planner)
      throw std::invalid_argument("One of the planners is null.");
  }
}

//==============================================================================
bool CompositePlanner::hasPlanner(const Planner& planner) const
{
  return std::find_if(
             mPlanners.begin(),
             mPlanners.end(),
             [&planner](const PlannerPtr& val) -> bool {
               return (val.get() == &planner);
             })
         != mPlanners.end();
}

//==============================================================================
bool CompositePlanner::canSolve(const Problem& problem) const
{
  for (const auto& planner : mPlanners)
  {
    if (planner->canSolve(problem))
      return true;
  }

  return false;
}

//==============================================================================
const std::vector<PlannerPtr>& CompositePlanner::getPlanners() const
{
  return mPlanners;
}

} // namespace planner
} // namespace aikido
