#include "aikido/planner/MetaPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
bool MetaPlanner::canSolve(const Problem* problem)
{
  for (const auto& planner : mPlanners)
  {
    if (planner->canSolve(problem))
      return true;
  }

  return false;
}

//==============================================================================
std::unordered_set<std::string> MetaPlanner::getSolvableProblems() const
{
  std::unordered_set<std::string> problems;

  for (const auto& planner : mPlanners)
  {
    const auto& subProblems = planner->getSolvableProblems();
    problems.insert(subProblems.begin(), subProblems.end());
  }

  return problems;
}

//==============================================================================
std::unordered_set<PlannerPtr> MetaPlanner::getPlannersCanSolve(
    const Problem* problem) const
{
  std::unordered_set<PlannerPtr> planners;

  for (const auto& planner : mPlanners)
  {
    auto metaPlanner = std::dynamic_pointer_cast<MetaPlanner>(planner);
    if (metaPlanner)
    {
      auto subPlanners = metaPlanner->getPlannersCanSolve(problem);
      planners.insert(subPlanners.begin(), subPlanners.end());
    }
    else
    {
      if (planner->canSolve(problem))
        planners.insert(planner);
    }
  }

  return planners;
}

} // namespace planner
} // namespace aikido
