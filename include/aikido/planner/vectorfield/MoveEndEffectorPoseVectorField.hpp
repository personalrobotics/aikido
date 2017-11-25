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
  /// \param[in] _goalPose Desired en-effector pose
  /// \param[in] _poseErrorTolerance Constraint error tolerance in meters
  /// \param[in] _initialStepSize Initial step size
  /// \param[in] _jointLimitTolerance If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0
  /// \param[in] _optimizationTolerance Tolerance on optimization
  MoveEndEffectorPoseVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      dart::dynamics::BodyNodePtr _bn,
      const Eigen::Isometry3d& _goalPose,
      double _poseErrorTolerance = 0.5,
      double _initialStepSize = 1e-2,
      double _jointLimitTolerance = 3e-2,
      double _optimizationTolerance = 5e-2);

  /// Vectorfield callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \param[out] _qd Joint velocities
  /// \return Whether joint velocities are successfully computed
  virtual bool getJointVelocities(
      const Eigen::VectorXd& _q, double _t, Eigen::VectorXd& _qd) override;

  /// Vectorfield planning status callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \return Status of planning
  virtual VectorFieldPlannerStatus checkPlanningStatus(
      const Eigen::VectorXd& _q, double _t) override;

protected:
  Eigen::Isometry3d mGoalPose;
  double mPoseErrorTolerance;
  double mInitialStepSize;
  double mJointLimitTolerance;
  double mOptimizationTolerance;

  Eigen::VectorXd mVelocityLowerLimits;
  Eigen::VectorXd mVelocityUpperLimits;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
