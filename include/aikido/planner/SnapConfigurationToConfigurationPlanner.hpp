#ifndef AIKIDO_PLANNER_SNAPCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
#define AIKIDO_PLANNER_SNAPCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_

#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace planner {

/// Planner that plans the straight-line trajectory to the goal.
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
class SnapConfigurationToConfigurationPlanner
    : public ConfigurationToConfigurationPlanner
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] interpolator Interpolator used to produce the output
  /// trajectory. If nullptr is passed in, GeodesicInterpolator is used by
  /// default.
  explicit SnapConfigurationToConfigurationPlanner(
      statespace::ConstStateSpacePtr stateSpace,
      statespace::ConstInterpolatorPtr interpolator = nullptr);

  /// Plans a trajectory from start state to goal state by using an interpolator
  /// to interpolate between them.
  ///
  /// The planner returns success if the resulting trajectory satisfies
  /// constraint at some resolution and failure (returning \c nullptr)
  /// otherwise. The reason for the failure is stored in the \c result output
  /// parameter.
  ///
  /// \param[in] problem Planning problem.
  /// \param[out] result Information about success or failure.
  /// \return Trajectory or \c nullptr if planning failed.
  /// \throw If \c problem is not ConfigurationToConfiguration.
  /// \throw If \c result is not ConfigurationToConfiguration::Result.
  trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr) override;

  /// Sets interpolator used to produce the output trajectory.
  void setInterpolator(statespace::ConstInterpolatorPtr interpolator);

  /// Returns the interpolator used to produce the output trajectory.
  statespace::ConstInterpolatorPtr getInterpolator() const;

  // Documentation inherited.
  std::shared_ptr<Planner> clone() const override;

protected:
  /// Interpolator used to produce the output trajectory.
  statespace::ConstInterpolatorPtr mInterpolator;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_SNAPCONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
