#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORALONGWORKSPACEPATHVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORALONGWORKSPACEPATHVECTORFIELD_HPP_

#include <aikido/statespace/SE3.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/planner/vectorfield/BodyNodePoseVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Vector field for moving end-effector by a direction and distance.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorAlongWorkspacePathVectorField
    : public BodyNodePoseVectorField
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace MetaSkeleton state space.
  /// \param[in] bn Body node of end-effector.
  /// \param[in] workspacePath A workspace trajectory.
  /// \param[in] positionTolerance Constraint tolerance in meters.
  /// \param[in] angularTolerance Constraint tolerance in radians.
  /// \param[in] tStep Time step to find vector tanget to current.
  ///  position on the trajectory, using finite differences.
  /// \param[in] jointLimitTolerance Padding to the boundary in meters
  /// \param[in] optimizationTolerance Tolerance on optimization
  /// \param[in] kpFF Feed-forward gain.
  ///  A 1x6 vector, where first 3 elements affect the translational
  /// velocity, the last 3 elements affect the rotational velocity.
  /// \param[in] kpE Error gain.
  ///  A 1x6 vector, where first 3 elements affect the translational
  ///  velocity, the last 3 elements affect the rotational velocity.
  MoveEndEffectorAlongWorkspacePathVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bn,
      aikido::trajectory::InterpolatedPtr workspacePath,
      double positionTolerance,
      double angularTolerance,
      double tStep,
      double initialStepSize,
      double jointLimitPadding,
      const Eigen::Vector6d& kpFF,
      const Eigen::Vector6d& kpE);

  // Documentation inherited.
  bool evaluateCartesianVelocity(
      const Eigen::Isometry3d& pose,
      Eigen::Vector6d& cartesianVelocity) const override;

  // Documentation inherited.
  VectorFieldPlannerStatus evaluateCartesianStatus(
      const Eigen::Isometry3d& pose) const override;

protected:
  /// Workspace path.
  aikido::trajectory::InterpolatedPtr mWorkspacePath;

  /// Position tolerance.
  double mPositionTolerance;

  /// Angular tolerance.
  double mAngularTolerance;

  /// Finite-difference step size.
  double mDeltaT;

  /// Initial step size of integrator.
  double mInitialStepSize;

  /// Joint limit padding.
  double mJointLimitPadding;

  /// Optimization tolerance.
  double mOptimizationTolerance;

  /// Feedforward gain.
  Eigen::Vector6d mKpFF;

  /// Error gain.
  Eigen::Vector6d mKpE;

  /// Start pose.
  Eigen::Isometry3d mStartPose;

  /// Workspace SE3
  std::shared_ptr<aikido::statespace::SE3> mSE3StateSpace;

  /// Timed workspace path.
  std::unique_ptr<aikido::trajectory::Interpolated> mTimedWorkspacePath;

  /// Duration of timed workspace path.
  double mDuration;

  /// Goal pose of timed workspace path.
  Eigen::Isometry3d mGoalPose;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
// AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORALONGWORKSPACEPATHVECTORFIELD_HPP_
