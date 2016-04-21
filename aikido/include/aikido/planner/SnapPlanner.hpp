#ifndef AIKIDO_PLANNER_SNAP_PLANNER_HPP_
#define AIKIDO_PLANNER_SNAP_PLANNER_HPP_
#include "../statespace/StateSpace.hpp"
#include "../statespace/Interpolator.hpp"
#include "../constraint/TestableConstraint.hpp"
#include "PlanningResult.hpp"

namespace aikido {
namespace planner {

/// Plan a trajectory from \c startState to \c goalState by using
/// \c interpolator to interpolate between them. The planner returns success if
/// the resulting trajectory satisfies \c constraint at some resolution and
/// failure (returning \c nullptr) otherwise. The reason for the failure is
/// stored in the \c planningResult output parameter.
///
/// \param startState start state
/// \param goalState goal state
/// \param stateSpace state space
/// \param constraint trajectory-wide constraint that must be satisfied
/// \param interpolator interpolator used to produce the output trajectory
/// \param[out] planningResult information about success or failure
/// \return trajectory or \c nullptr if planning failed
path::PiecewiseLinearTrajectoryPtr planSnap(
  const statespace::StateSpace::State *startState,
  const statespace::StateSpace::State *goalState,
  const std::shared_ptr<statespace::StateSpace> stateSpace,
  const std::shared_ptr<constraint::TestableConstraint> constraint,
  const std::shared_ptr<statespace::Interpolator> interpolator,
  planner::PlanningResult *planningResult);

} // namespace planner
} // namespace aikido

#endif  // AIKIDO_PLANNER_SNAP_PLANNER_HPP_
