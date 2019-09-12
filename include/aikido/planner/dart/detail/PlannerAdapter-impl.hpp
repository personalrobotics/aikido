#include <type_traits>

#include "aikido/planner/dart/PlannerAdapter.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
template <
    typename DelegatePlanner,
    typename TargetPlanner,
    typename DelegateIsDartPlanner>
PlannerAdapter<DelegatePlanner, TargetPlanner, DelegateIsDartPlanner>::
    PlannerAdapter(
        std::shared_ptr<DelegatePlanner> planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : TargetPlanner(
        std::dynamic_pointer_cast<
            const statespace::dart::MetaSkeletonStateSpace>(
            planner->getStateSpace()),
        std::move(metaSkeleton))
  , mDelegate(std::move(planner))
{
  // Do nothing
}

//==============================================================================
template <typename DelegatePlanner, typename TargetPlanner>
PlannerAdapter<
    DelegatePlanner,
    TargetPlanner,
    typename std::enable_if<std::is_base_of<
        dart::SingleProblemPlanner<
            DelegatePlanner,
            typename DelegatePlanner::SolvableProblem>,
        DelegatePlanner>::value>::type>::
    PlannerAdapter(std::shared_ptr<DelegatePlanner> planner)
  : TargetPlanner(
        planner->getMetaSkeletonStateSpace(), planner->getMetaSkeleton())
  , mDelegate(std::move(planner))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
