#ifndef AIKIDO_PLANNER_SNAPPLANNER_HPP_
#define AIKIDO_PLANNER_SNAPPLANNER_HPP_

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/ConcretePlanner.hpp"
#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/ConfigurationToConfigurations.hpp"
#include "aikido/planner/ConfigurationToTSR.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planner that checks the straight-line trajectory to the goal.
///
/// SnapPlanner is a utility planner class that collision checks the
/// straight-line trajectory to the goal. If that trajectory is invalid, e.g.,
/// due to an environment or self collision, the planner immediately returns
/// \c nullptr.
///
/// SnapPlanner is intended to be used only as a "short circuit" to speed-up
/// planning between nearby configurations. This planner is most commonly used
/// as the first item in a Sequence meta-planner to avoid calling a motion
/// planner when the trivial solution is valid.
class SnapPlanner : public ConcretePlanner
{
public:
  /// Constructor
  SnapPlanner();

  /// Solves ConfigurationToConfiguration problem.
  ///
  /// The planner returns success if the resulting trajectory satisfies
  /// constraint at some resolution and failure (returning \c nullptr)
  /// otherwise. The reason for the failure is stored in the \c result output
  /// parameter.
  ///
  /// \param problem Planning problem.
  /// \param[out] result Information about success or failure.
  /// \return Trajectory or \c nullptr if planning failed.
  /// \throw If \c problem is not ConfigurationToConfiguration.
  /// \throw If \c result is not ConfigurationToConfiguration::Result.
  trajectory::InterpolatedPtr planToConfiguration(
      const Problem* problem, Problem::Result* result = nullptr);

protected:
  // Documentation inherited.
  PlanningFunctionMap& getPlanningFunctionMap() override;

  /// Whether planning function map is set.
  static bool mIsRegisteredPlanningFunctions;

  /// Planning function map.
  static PlanningFunctionMap mPlanningFunctionMap;
};

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
