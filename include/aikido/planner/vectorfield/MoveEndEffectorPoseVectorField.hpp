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
  /// Constructor
  ///
  /// \param[in] _stateSpace MetaSkeleton state space
  /// \param[in] _bn Body node of end-effector
  /// \param[in] _goalPose Desired end-effector pose
  /// \param[in] _poseErrorTolerance Constraint error tolerance in meters
  /// \param[in] _linearVelocityGain Linear velocity gain in workspace
  /// \param[in] _angularVelocityGain Angular velocity gain in workspace
  /// \param[in] _initialStepSize Initial step size
  /// \param[in] _jointLimitPadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0
  /// \param[in] _optimizationTolerance Tolerance on optimization
  MoveEndEffectorPoseVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      dart::dynamics::BodyNodePtr _bn,
      const Eigen::Isometry3d& _goalPose,
      double _linearVelocityGain = 1.0,
      double _angularVelocityGain = 1.0,
      double _poseErrorTolerance = 0.5,
      double _initialStepSize = 1e-1,
      double _jointLimitPadding = 3e-2,
      double _optimizationTolerance = 5e-2);

  /// Vectorfield callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \param[out] _qd Joint velocities
  /// \return Whether joint velocities are successfully computed
  bool getJointVelocities(Eigen::VectorXd& _qd) const override;

  /// Vectorfield planning status callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \return Status of planning
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

  /// Padding of joint limits
  double mJointLimitPadding;

  /// Tolerance of optimization solver.
  double mOptimizationTolerance;

  Eigen::VectorXd mVelocityLowerLimits;
  Eigen::VectorXd mVelocityUpperLimits;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
