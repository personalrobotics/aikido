#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Vector field for moving end-effector by a direction and distance.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorPoseVectorField : public ConfigurationSpaceVectorField
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
  /// \param[in] optimizationTolerance Tolerance on optimization.
  MoveEndEffectorPoseVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bn,
      const Eigen::Isometry3d& goalPose,
      double poseErrorTolerance = 0.5,
      double linearVelocityGain = 1.0,
      double angularVelocityGain = 1.0,
      double initialStepSize = 1e-1,
      double jointLimitPadding = 3e-2,
      double optimizationTolerance = 5e-2);

  /// Vectorfield callback function.
  ///
  /// \param[out] qd Joint velocities.
  /// \return Whether joint velocities are successfully computed.
  bool getJointVelocities(Eigen::VectorXd& qd) const override;

  /// Vectorfield planning status callback function
  ///
  /// \return Status of planning.
  VectorFieldPlannerStatus checkPlanningStatus() const override;

protected:
  /// Goal pose.
  Eigen::Isometry3d mGoalPose;

  /// Tolerance of pose error.
  double mPoseErrorTolerance;

  /// Linear velocity gain.
  double mLinearVelocityGain;

  /// Angular velocit gain.
  double mAngularVelocityGain;

  /// Initial step size of adaptive integration.
  double mInitialStepSize;

  /// Padding of joint limits.
  double mJointLimitPadding;

  /// Tolerance of optimization solver.
  double mOptimizationTolerance;

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
