#include "aikido/planner/AdaptedPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
template <typename DelegatePlanner>
AdaptedPlanner<DelegatePlanner>::AdaptedPlanner(
    std::shared_ptr<DelegatePlanner> planner)
  : Planner(planner->getStateSpace()), mDelegate(std::move(planner))
{
  // Do nothing.
}

} // namespace planner
} // namespace aikido
