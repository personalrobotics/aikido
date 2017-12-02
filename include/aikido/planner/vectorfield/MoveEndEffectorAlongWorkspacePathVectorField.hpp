#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORALONGWORKSPACEPATHVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORALONGWORKSPACEPATHVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Vector field for moving end-effector by a direction and distance.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorAlongWorkspacePathVectorField
    : public ConfigurationSpaceVectorField
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
      double positionTolerance = 0.01,
      double angularTolerance = 0.15,
      double tStep = 0.001,
      double initialStepSize = 1e-1,
      double jointLimitPadding = 3e-2,
      double optimizationTolerance = 1e-3,
      const Eigen::Vector6d& kpFF = Eigen::VectorXd::Constant(6, 0.4),
      const Eigen::Vector6d& kpE = Eigen::VectorXd::Constant(6, 1.0));

  /// Vectorfield callback function.
  ///.
  /// \param[out] qd Joint velocities.
  /// \return Whether joint velocities are successfully computed.
  bool getJointVelocities(Eigen::VectorXd& qd) const override;

  /// Vectorfield planning status callback function.
  ///
  /// Fail if deviation larger than position and angular tolerance.
  /// Succeed if distance moved is larger than max_distance.
  /// Cache and continue if distance moved is larger than distance.
  /// \return Status of planning.
  VectorFieldPlannerStatus checkPlanningStatus() const override;

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
