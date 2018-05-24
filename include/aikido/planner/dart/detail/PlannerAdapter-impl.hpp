#include "aikido/planner/dart/PlannerAdapter.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
template <typename DelegatePlanner, typename TargetPlanner>
PlannerAdapter<DelegatePlanner, TargetPlanner>::PlannerAdapter(
    std::shared_ptr<DelegatePlanner> planner)
  : TargetPlanner(
        planner->getMetaSkeletonStateSpace(), planner->getMetaSkeleton())
  , mDelegate(std::move(planner))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
