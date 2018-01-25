#include "aikido/planner/SequenceMetaPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
trajectory::TrajectoryPtr SequenceMetaPlanner::solve(
    const Problem* problem, Problem::Result* result)
{
  for (const auto& planner : mPlanners)
  {
    auto trajectory = planner->solve(problem, result);
    if (trajectory)
      return trajectory;
  }

  return nullptr;
}

} // namespace planner
} // namespace aikido
