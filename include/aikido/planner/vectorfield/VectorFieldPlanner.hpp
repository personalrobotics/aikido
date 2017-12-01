#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_

#include <functional>
#include <dart/common/Timer.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorAlongWorkspacePathVectorField.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorPoseVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// VectorField Planner generates a trajectory by following a vector field
/// defined in joint space.
/// A planned trajectroy ends either by a predefined termination criterion or a
/// integral time.
///
/// This class defines two callback functions for a integrator.
/// step() provides joint velocities the vector field planner should follow,
/// check() is called after each integration step to check planner status.
class VectorFieldPlanner
{
public:
  /// Constructor.
  ///
  /// \param[in] vectorField Vector field in configuration space.
  /// \param[in] constraint Constraint to be satisfied.
  /// \param[in] initialStepSize Initial step size in integation.
  VectorFieldPlanner(
      const ConfigurationSpaceVectorFieldPtr vectorField,
      const aikido::constraint::TestablePtr constraint,
      double initialStepSize = 0.1);

  /// Generate a trajectory following the vector field along given time.
  ///
  /// \param[in] integrationTimeInterval Position in configuration space.
  /// \param[in] timelimit Timelimit for integration calculation.
  /// \param[in] useCollisionChecking Whether collision checking is
  /// considered in planning.
  /// \param[in] useDofLimitChecking Whether Dof Limits are considered
  /// in planning.
  /// \return A trajectory following the vector field.
  std::unique_ptr<aikido::trajectory::Spline> plan(
      double integrationTimeInterval,
      double timelimit,
      double useCollisionChecking = true,
      double useDofLimitChecking = true);

protected:
  /// Vectorfield callback function that returns joint velocities for
  /// integration.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[out] qd Joint velocities in configuration space.
  /// \param[in] t Current time being planned.
  void step(const Eigen::VectorXd& q, Eigen::VectorXd& qd, double t);

  /// Check status after every intergration step.
  ///
  /// \param[in] q Position in configuration space.
  /// \param[in] t Current time being planned.
  void check(const Eigen::VectorXd& q, double t);

  /// Vector field.
  ConfigurationSpaceVectorFieldPtr mVectorField;

  /// Planning constraint.
  aikido::constraint::TestablePtr mConstraint;

  /// Meta Skeleton state space.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Meta skeleton.
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// Body node of end-effector.
  dart::dynamics::BodyNodePtr mBodyNode;

  /// Timer for timelimit.
  dart::common::Timer mTimer;

  /// Planning timelimit.
  double mTimelimit;
  std::vector<Knot> mKnots;

  /// Initial step size for adaptive integrator.
  double mInitialStepSize;

  /// Cached index of knots.
  int mCacheIndex;

  /// current index of knots.
  int mIndex;

  /// Enable collision checking in following vector field.
  bool mEnableCollisionCheck;

  /// Enable DOF limit checking in following vector field.
  bool mEnableDofLimitCheck;
};

/// Plan to a trajectory that moves the end-effector by a given direction and
/// distance.
///
/// \param[in] stateSpace MetaSkeleton state space.
/// \param[in] bn Body node of the end-effector.
/// \param[in] constraint Trajectory-wide constraint that must be satisfied.
/// \param[in] direction Direction of moving the end-effector.
/// \param[in] distance  Distance of moving the end-effector.
/// \param[in] maxDistance Max distance of moving the end-effector.
/// \param[in] positionTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance.
/// \param[in] angularTolerance How a planned trajectory is allowed to deviate
/// from a given direction.
/// \param[in] linearVelocityGain Linear velocity gain in workspace.
/// \param[in] useCollisionChecking Whether collision checking is
/// considered in planning.
/// \param[in] useDofLimitChecking Whether Dof Limits are considered
/// in planning.
/// \param[in] initialStepSize Initial step size.
/// \param[in] jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0.
/// \param[in] optimizationTolerance Tolerance on optimization.
/// \param[in] timelimit timeout in seconds.
/// \param[in] integralTimeInterval The time interval to integrate over
/// \return Trajectory or \c nullptr if planning failed.
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Vector3d& direction,
    double distance,
    double maxDistance = MoveEndEffectorOffsetVectorField::InvalidMaxDistance,
    double positionTolerance = 0.01,
    double angularTolerance = 0.15,
    double linearVelocityGain = 1.0,
    bool useCollisionChecking = true,
    bool useDofLimitChecking = true,
    double initialStepSize = 1e-2,
    double jointLimitTolerance = 3e-2,
    double optimizationTolerance = 1e-3,
    double timelimit = 5.0,
    double integralTimeInterval = 10.0);

/// Plan to an end effector pose by following a geodesic loss function
/// in SE(3) via an optimized Jacobian.
///
/// \param[in] stateSpace MetaSkeleton state space.
/// \param[in] bn Body node of the end-effector.
/// \param[in] constraint Trajectory-wide constraint that must be satisfied.
/// \param[in] goalPose Desired end-effector pose.
/// \param[in] positionErrorTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance.
/// \param[in] linearVelocityGain Linear velocity gain in workspace.
/// \param[in] angularVelocityGain Angular velocity gain in workspace.
/// \param[in] useCollisionChecking Whether collision checking is
/// considered in planning.
/// \param[in] useDofLimitChecking Whether Dof Limits are considered
/// in planning.
/// \param[in] initialStepSize Initial step size.
/// \param[in] jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0.
/// \param[in] optimizationTolerance Tolerance on optimization.
/// \param[in] timelimit timeout in seconds.
/// \param[in] integralTimeInterval The time interval to integrate over.
/// \return Trajectory or \c nullptr if planning failed.
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorPose(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Isometry3d& goalPose,
    double poseErrorTolerance,
    double linearVelocityGain = 1.0,
    double angularvelocityGain = 1.0,
    bool useCollisionChecking = true,
    bool useDofLimitChecking = true,
    double initialStepSize = 1e-2,
    double jointLimitTolerance = 3e-2,
    double optimizationTolerance = 1.,
    double timelimit = 5.0,
    double integralTimeInterval = 10.0);

/// Plan to an end effector pose by following a workspace path
/// in SE(3) via an optimized Jacobian.
///
/// \param[in] stateSpace MetaSkeleton state space
/// \param[in] bn Body node of the end-effector
/// \param[in] constraint Trajectory-wide constraint that must be satisfied
/// \param[in] workspacePath A workspace trajectory
/// \param[in] positionErrorTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance
/// \param[in] tStep Time step to find vector tanget to current
/// position on the trajectory, using finite differences.
/// \param[in] kpFF Feed-forward gain.
///  A 1x6 vector, where first 3 elements affect the translational
/// velocity, the last 3 elements affect the rotational velocity.
/// \param[in] kpE Error gain.
///  A 1x6 vector, where first 3 elements affect the translational
///  velocity, the last 3 elements affect the rotational velocity.
/// \param[in] useCollisionChecking Whether collision checking is
/// considered in planning
/// \param[in] useDofLimitChecking Whether Dof Limits are considered
/// in planning
/// \param[in] initialStepSize Initial step size
/// \param[in] jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0
/// \param[in] optimizationTolerance Tolerance on optimization
/// \param[in] timelimit timeout in seconds
/// \param[in] integralTimeInterval The time interval to integrate over
/// \return Trajectory or \c nullptr if planning failed
std::unique_ptr<aikido::trajectory::Spline> planWorkspacePath(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const aikido::trajectory::InterpolatedPtr workspacePath,
    double positionTolerance = 0.01,
    double angularTolerance = 0.15,
    double tStep = 0.001,
    const Eigen::Vector6d& kpFF = Eigen::VectorXd::Constant(6, 0.4),
    const Eigen::Vector6d& kpE = Eigen::VectorXd::Constant(6, 1.0),
    bool useCollisionChecking = true,
    bool useDofLimitChecking = true,
    double initialStepSize = 1e-3,
    double jointLimitTolerance = 3e-2,
    double optimizationTolerance = 10.,
    double timelimit = 5.0,
    double integralTimeInterval = 10.0);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
