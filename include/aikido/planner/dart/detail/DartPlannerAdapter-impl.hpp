#include "aikido/planner/dart/DartPlannerAdapter.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
template <typename DelegatePlanner, typename TargetPlanner>
DartPlannerAdapter<DelegatePlanner, TargetPlanner>::DartPlannerAdapter(
    std::shared_ptr<DelegatePlanner> planner,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : TargetPlanner(
        std::dynamic_pointer_cast<const aikido::statespace::dart::
                                      MetaSkeletonStateSpace>(
            planner->getStateSpace()),
        std::move(metaSkeleton))
  , mDelegate(std::move(planner))
{
  // Do nothing
}

//==============================================================================
template <typename DelegatePlanner, typename TargetPlanner>
trajectory::TrajectoryPtr
DartPlannerAdapter<DelegatePlanner, TargetPlanner>::plan(
    const typename DelegatePlanner::SolvableProblem& problem,
    Planner::Result* result)
{
  return mDelegate->plan(problem, result);
}

} // namespace dart
} // namespace planner
} // namespace aikido
