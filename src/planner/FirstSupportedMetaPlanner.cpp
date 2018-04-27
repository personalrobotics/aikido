#include "aikido/planner/FirstSupportedMetaPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
trajectory::TrajectoryPtr FirstSupportedMetaPlanner::plan(
    const Problem& problem, Result* result)
{
  for (const auto& planner : mPlanners)
  {
    if (!planner->canSolve(problem))
      continue;

    return planner->plan(problem, result);
  }

  // TODO: Set result "no supported planner".

  return nullptr;
}

} // namespace planner
} // namespace aikido
