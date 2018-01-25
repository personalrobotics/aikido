#include "aikido/planner/FirstSupportedMetaPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
trajectory::TrajectoryPtr FirstSupportedMetaPlanner::solve(
    const Problem* problem, Problem::Result* result)
{
  for (const auto& planner : mPlanners)
  {
    if (!planner->canSolve(problem))
      continue;

    return planner->solve(problem, result);
  }

  // TODO: Set result "no supported planner".

  return nullptr;
}

} // namespace planner
} // namespace aikido
