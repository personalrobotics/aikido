#ifndef AIKIDO_PLANNER_SNAPPLANNER_HPP_
#define AIKIDO_PLANNER_SNAPPLANNER_HPP_

#include <memory>

#include "aikido/common/deprecated.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/statespace/Interpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Plan a trajectory from \c startState to \c goalState by using
/// \c interpolator to interpolate between them. The planner returns success if
/// the resulting trajectory satisfies \c constraint at some resolution and
/// failure (returning \c nullptr) otherwise. The reason for the failure is
/// stored in the \c planningResult output parameter.
///
/// \param stateSpace state space
/// \param startState start state
/// \param goalState goal state
/// \param interpolator interpolator used to produce the output trajectory
/// \param constraint trajectory-wide constraint that must be satisfied
/// \param[out] planningResult information about success or failure
/// \return trajectory or \c nullptr if planning failed
///
/// \deprecated Deprecated in 0.3. Please use
/// SnapConfigurationToConfigurationPlanner instead.
AIKIDO_DEPRECATED(0.3)
trajectory::InterpolatedPtr planSnap(
    const statespace::ConstStateSpacePtr& stateSpace,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    const std::shared_ptr<statespace::Interpolator>& interpolator,
    const std::shared_ptr<constraint::Testable>& constraint,
    planner::PlanningResult& planningResult);

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_SNAPPLANNER_HPP_
