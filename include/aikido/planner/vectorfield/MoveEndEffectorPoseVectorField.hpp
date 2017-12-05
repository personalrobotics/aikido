#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/BodyNodePoseVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Vector field for moving end-effector by a direction and distance.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorPoseVectorField : public BodyNodePoseVectorField
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace MetaSkeleton state space.
  /// \param[in] bn Body node of end-effector.
  /// \param[in] goalPose Desired end-effector pose.
  /// \param[in] poseErrorTolerance Constraint error tolerance in meters.
  /// \param[in] linearVelocityGain Linear velocity gain in workspace.
  /// \param[in] angularVelocityGain Angular velocity gain in workspace.
  /// \param[in] initialStepSize Initial step size.
  /// \param[in] jointLimitPadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  MoveEndEffectorPoseVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bn,
      const Eigen::Isometry3d& goalPose,
      double poseErrorTolerance = 0.5,
      double linearVelocityGain = 1.0,
      double angularVelocityGain = 1.0,
      double initialStepSize = 5e-2,
      double jointLimitPadding = 3e-2);

  // Documentation inherited.
  bool evaluateVelocity(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& qd) const override;

  // Documentation inherited.
  VectorFieldPlannerStatus evaluateStatus(
      const aikido::statespace::StateSpace::State* state) const override;

protected:
  /// Goal pose.
  Eigen::Isometry3d mGoalPose;

  /// Tolerance of pose error.
  double mPoseErrorTolerance;

  /// Linear velocity gain.
  double mLinearVelocityGain;

  /// Angular velocit gain.
  double mAngularVelocityGain;

  /// Initial step size of integrator.
  double mInitialStepSize;

  /// Padding of joint limits.
  double mJointLimitPadding;

  /// Joint velocities lower limits.
  Eigen::VectorXd mVelocityLowerLimits;

  /// Joint velocities upper limits.
  Eigen::VectorXd mVelocityUpperLimits;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
