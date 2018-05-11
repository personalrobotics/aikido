#include "aikido/planner/SequenceMetaPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
trajectory::TrajectoryPtr SequenceMetaPlanner::plan(
    const Problem& problem, Result* result)
{
  for (const auto& planner : mPlanners)
  {
    auto trajectory = planner->plan(problem, result);
    if (trajectory)
      return trajectory;
  }

  return nullptr;
}

} // namespace planner
} // namespace aikido
