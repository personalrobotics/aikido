#ifndef AIKIDO_PLANNER_SNAP_PLANNER_HPP_
#define AIKIDO_PLANNER_SNAP_PLANNER_HPP_

#include <memory>
#include <exception>
#include <stdexcept>
#include <vector>

namespace aikido {
namespace planner {

path::PiecewiseLinearTrajectoryPtr planSnap(
  const aikido::statespace::StateSpace::State *startState,
  const aikido::statespace::StateSpace::State *goalState,
  const std::shared_ptr<aikido::statespace::StateSpace> stateSpace,
  const std::shared_ptr<aikido::constraint::TestableConstraint> constraint,
  const std::shared_ptr<aikido::statespace::Interpolator> interpolator,
  aikido::planner::PlanningResult *planningResult);

} // namespace planner
} // namespace aikido

#endif  // AIKIDO_PLANNER_SNAP_PLANNER_HPP_
