#include "aikido/planner/MetaPlanner.hpp"

#include <algorithm>
#include <dart/common/Console.hpp>

namespace aikido {
namespace planner {

//==============================================================================
MetaPlanner::MetaPlanner(const std::vector<PlannerPtr>& planners)
  : mPlanners(planners)
{
  // Do nothing
}

//==============================================================================
void MetaPlanner::addPlanner(PlannerPtr planner)
{
  if (std::find(mPlanners.begin(), mPlanners.end(), planner) != mPlanners.end())
  {
    dtwarn << "[MetaPlanner::addPlanner] Attempts to add a planner that "
              "already exists in this MetaPlanner.\n";
    return;
  }

  mPlanners.emplace_back(std::move(planner));
}

//==============================================================================
bool MetaPlanner::hasPlanner(const Planner* planner)
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
const std::vector<PlannerPtr>& MetaPlanner::getPlanners() const
{
  return mPlanners;
}

//==============================================================================
PlannerPtr MetaPlanner::getPlanner(std::size_t index)
{
  if (index > mPlanners.size())
    throw std::invalid_argument("index is out of bound");

  return mPlanners[index];
}

//==============================================================================
bool MetaPlanner::canSolve(const Problem* problem) const
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
