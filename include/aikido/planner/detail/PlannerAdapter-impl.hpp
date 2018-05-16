#include "aikido/planner/PlannerAdapter.hpp"

namespace aikido {
namespace planner {

//==============================================================================
template <typename DelegatePlanner>
PlannerAdapter<DelegatePlanner>::PlannerAdapter(
    std::shared_ptr<DelegatePlanner> planner)
  : Planner(planner->getStateSpace()), mDelegate(std::move(planner))
{
  // Do nothing.
}

} // namespace planner
} // namespace aikido
