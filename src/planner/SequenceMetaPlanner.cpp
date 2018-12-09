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

//==============================================================================
PlannerPtr SequenceMetaPlanner::clone(common::RNG* rng) const
{
  throw std::runtime_error("Cloning MetaPlanner is not suported.");
}

//==============================================================================
bool SequenceMetaPlanner::stopPlanning()
{
  return false;
}

} // namespace planner
} // namespace aikido
