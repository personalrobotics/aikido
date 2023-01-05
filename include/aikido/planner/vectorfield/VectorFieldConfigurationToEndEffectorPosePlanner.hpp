#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDCONFIGURATIONTOENDEFFECTORPOSEPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDCONFIGURATIONTOENDEFFECTORPOSEPLANNER_HPP_

#include "aikido/planner/dart/ConfigurationToEndEffectorPose.hpp"
#include "aikido/planner/dart/ConfigurationToEndEffectorPosePlanner.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

/// Planner that generates a trajectory that moves the end-effector to
/// a given pose.
class VectorFieldConfigurationToEndEffectorPosePlanner
  : public planner::dart::ConfigurationToEndEffectorPosePlanner
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space that this planner is associated with.
  /// \param[in] metaSkeleton MetaSkeleton to plan with.
  /// \param[in] goalTolerance Distance from goal to stop planning.
  /// \param[in] anglePoseRatio Conversion ratio from radian to
  /// meter in computing the goal tolerance.
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
  VectorFieldConfigurationToEndEffectorPosePlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      double goalTolerance,
      double anglePoseRatio,
      double positionTolerance,
      double angularTolerance,
      double initialStepSize,
      double jointLimitTolerance,
      double constraintCheckResolution,
      std::chrono::duration<double> timelimit);

  /// Plan to a trajectory that moves the end-effector to a
  /// given pose.
  ///
  /// The planner returns success if the resulting trajectory satisfies
  /// constraint at some resolution and failure (returning \c nullptr)
  /// otherwise. The reason for the failure is stored in the \c result output
  /// parameter.
  ///
  /// \param[in] problem Planning problem.
  /// \param[out] result Information about success or failure.
  /// \return Trajectory or \c nullptr if planning failed.
  trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr) override;

protected:
  /// Tolerated geodesic distance from goal.
  double mGoalTolerance;

  /// Conversion ratio from radian to
  /// meter in computing the combined geodesic error.
  double mAnglePoseRatio;

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

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDCONFIGURATIONTOENDEFFECTORPOSEPLANNER_HPP_
