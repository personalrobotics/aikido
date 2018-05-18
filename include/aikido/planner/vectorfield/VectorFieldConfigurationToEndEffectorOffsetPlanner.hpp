#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDCONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDCONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_

#include "aikido/planner/ConfigurationToEndEffectorOffset.hpp"
#include "aikido/planner/ConfigurationToEndEffectorOffsetPlanner.hpp"
#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"

namespace aikido {
namespace planner {

/// Planner that generates a trajectory that moves the end-effector by a given
/// direction and distance.
class VectorFieldConfigurationToEndEffectorOffsetPlanner
    : public ConfigurationToEndEffectorOffsetPlanner
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space that this planner is associated with.
  /// \param[in] metaskeleton MetaSkeleton to plan with.
  /// \param[in] distanceTolerance How much a planned trajectory is allowed to
  /// deviate from the requested distance to move the end-effector.
  /// \param[in] positionTolerance How a planned trajectory is allowed to
  /// deviated from a straight line segment defined by the direction and the
  /// distance.
  /// \param[in] angularTolerance How a planned trajectory is allowed to deviate
  /// from a given direction.
  /// \param[in] initialStepSize Initial step size.
  /// \param[in] jointLimitTolerance If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  /// \param[in] constraintCheckResolution Resolution used in constraint
  /// checking.
  /// \param[in] timelimit timeout in seconds.
  VectorFieldConfigurationToEndEffectorOffsetPlanner(
      statespace::ConstStateSpacePtr stateSpace,
      dart::dynamics::MetaSkeletonPtr metaskeleton,
      double distanceTolerance,
      double positionTolerance,
      double angularTolerance,
      double initialStepSize,
      double jointLimitTolerance,
      double constraintCheckResolution,
      std::chrono::duration<double> timelimit);

  /// Plan to a trajectory that moves the end-effector by a given direction and
  /// distance.
  ///
  /// The planner returns success if the resulting trajectory satisfies
  /// constraint at some resolution and failure (returning \c nullptr)
  /// otherwise. The reason for the failure is stored in the \c result output
  /// parameter.
  ///
  /// \param[in] problem Planning problem.
  /// \param[out] result Information about success or failure.
  /// \return Trajectory or \c nullptr if planning failed.
  /// \throw If \c problem is not ConfigurationToEndEffectorOffset.
  /// \throw If \c result is not ConfigurationToEndEffectorOffset::Result.
  /// \throw If mStateSpace is not MetaSkeletonStateSpace.
  trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr) override;

protected:
  /// MetaSkeleton to plan with.
  dart::dynamics::MetaSkeletonPtr mMetaskeleton;

  /// How much a planned trajectory is allowed to deviate from the requested
  /// distance to move the end-effector.
  double mDistanceTolerance;

  /// How a planned trajectory is allowed to deviated from a straight line
  /// segment defined by the direction and the distance.
  double mPositionTolerance;

  /// How a planned trajectory is allowed to deviate from a given direction.
  double mAngularTolerance;

  /// Initial step size.
  double mInitialStepSize;

  /// If less then this distance to joint limit, velocity is bounded in that
  /// direction to 0.
  double mJointLimitTolerance;

  /// Resolution used in constraint checking.
  double mConstraintCheckResolution;

  /// Timeout in seconds.
  std::chrono::duration<double> mTimelimit;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDCONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_
