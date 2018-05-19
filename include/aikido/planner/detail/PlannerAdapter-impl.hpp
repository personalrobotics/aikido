#include "aikido/planner/PlannerAdapter.hpp"

namespace aikido {
namespace planner {

//==============================================================================
template <typename DelegatePlanner, typename TargetPlanner>
PlannerAdapter<DelegatePlanner, TargetPlanner>::PlannerAdapter(
    std::shared_ptr<DelegatePlanner> planner)
  : TargetPlanner(planner->getStateSpace()), mDelegate(std::move(planner))
{
  // Do nothing
}

} // namespace planner
} // namespace aikido
