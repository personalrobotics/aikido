#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Solver.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

struct Knot
{
  /// Timestamp.
  double mT;

  /// Positions.
  Eigen::VectorXd mPositions;

  /// Velocities.
  Eigen::VectorXd mVelocities; // Not used in current implementation
};

/// Convert a sequence of knots into a Spline trajectory.
///
/// \param[in] knots A sequence of knots.
/// \param[in] cache_index Total cache index number.
/// \param[in] stateSpace MetaSkeleton state space.
/// \return A Spline trajectory
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    int cacheIndex,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace);

/// A function class that defines an objective. The objective measures
/// the difference between a desired twist and Jacobian * joint velocities.
///
class DesiredTwistFunction : public dart::optimizer::Function
{
public:
  using Twist = Eigen::Vector6d;
  using Jacobian = dart::math::Jacobian;

  /// Constructor.
  ///
  /// \param[in] twist A desired twist.
  /// \param[in] jacobian System Jacobian.
  DesiredTwistFunction(const Twist& twist, const Jacobian& jacobian);

  /// Implementation inherited.
  /// Evaluating an objective by a state value.
  ///
  /// \param[in] qd Joint velocities.
  /// \return Objective value.
  double eval(const Eigen::VectorXd& qd) override;

  /// Implementation inherited.
  /// Evaluating gradient of an objective by a state value.
  /// \param[in] qd Joint velocities.
  /// \param[out] grad Gradient of a defined objective
  void evalGradient(
      const Eigen::VectorXd& qd, Eigen::Map<Eigen::VectorXd> grad) override;

private:
  /// Twist.
  Twist mTwist;

  /// Jacobian of Meta Skeleton
  Jacobian mJacobian;
};

/// Compute joint velocity from a given twist.
///
/// \param[out] jointVelocity Calculated joint velocities.
/// \param[in] desiredTwist Desired twist, which consists of angular velocity
/// and linear velocity.
/// \param[in] stateSpace MetaSkeleton state space.
/// \param[in] bodyNode Body node of the end-effector.
/// \param[in] jointLimitPadding If less then this distance to joint
/// limit, velocity is bounded in that direction to 0.
/// \param[in] jointVelocityLowerLimits Joint velocity lower bounds.
/// \param[in] jointVelocityUpperLimits Joint velocity upper bounds.
/// \param[in] jointVelocityLimited Whether joint velocity bounds are
/// considered in optimization.
/// \param[in] maxStepSize Max step size in second.
/// \param[in] optimizationTolerance  Callback of vector field calculation.
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& jointVelocity,
    const Eigen::Vector6d& desiredTwist,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const dart::dynamics::BodyNodePtr bodyNode,
    double jointLimitPadding,
    const Eigen::VectorXd& jointVelocityLowerLimits,
    const Eigen::VectorXd& jointVelocityUpperLimits,
    bool jointVelocityLimited,
    double maxStepSize,
    double optimizationTolerance);

/// Compute the twist in global coordinate that corresponds to the gradient of
/// the geodesic distance between two transforms.
///
/// \param[in] currentTrans Current transformation.
/// \param[in] goalTrans Goal transformation.
Eigen::Vector6d computeGeodesicTwist(
    const Eigen::Isometry3d& currentTrans, const Eigen::Isometry3d& goalTrans);

/// Compute the error in gloabl coordinate between two transforms.
///
/// \param[in] currentTrans Current transformation.
/// \param[in] goalTrans Goal transformation.
Eigen::Vector4d computeGeodesicError(
    const Eigen::Isometry3d& currentTrans, const Eigen::Isometry3d& goalTrans);

/// Compute the geodesic distance between two transforms.
/// gd = norm( relative translation + r * axis-angle error )
///
/// \param[in] currentTrans Current transformation.
/// \param[in] goalTrans Goal transformation.
/// \param[in] r In units of meters/radians converts radians to meters.
double computeGeodesicDistance(
    const Eigen::Isometry3d& currentTrans,
    const Eigen::Isometry3d& goalTrans,
    double r = 1.0);

/// Get a Transform in a timed SE3 trajectory at time t
///
/// \param[in] stateSpace SE3 state space
/// \param[in] timedSE3Path A timed SE3 trajectory
/// \param[in] t Time to query
/// \param[out] trans Found transform
/// \return Whether a transform is found
bool getTransfromFromTimedSE3Trajectory(
    std::shared_ptr<aikido::statespace::SE3> stateSpace,
    const aikido::trajectory::Interpolated* timedSE3Path,
    double t,
    Eigen::Isometry3d& trans);

/// Find the location on a workspace trajectory which is closest
/// to the specified transform.
///
/// \param[in] currentPose A 4x4 transformation matrix.
/// \param[in] timedWorkspacePath A timed workspace trajectory.
/// \param[in] dt Resolution at which to sample along the trajectory.
/// \param[out] minDist The minimum distance.
/// \param[out] tLoc The time value along the timed trajectory.
/// \param[out] transLoc The transform.
bool getMinDistanceBetweenTransformAndWorkspaceTraj(
    const Eigen::Isometry3d& currentPose,
    const aikido::trajectory::Interpolated* timedWorkspacePath,
    const std::shared_ptr<aikido::statespace::SE3> SE3StateSpace,
    double dt,
    double& minDist,
    double& tLoc,
    Eigen::Isometry3d& transLoc);

/// Compute the geodesic unit velocity timing of a workspace path or
/// trajectory, also called a path length parameterization.
/// The path length is calculated as the sum of all segment lengths,
/// where each segment length = norm( delta_translation^2 +
///                                       alpha^2*delta_orientation^2 )
/// Note: Currently only linear velocity interpolation is supported,
///      however OpenRAVE does allow you to specify quadratic
///      interpolation.
/// \param[in] untimedTraj Workspace path or trajectory.
/// \param[in] SE3StateSpace SE3 state space.
/// \param[in] alpha Weighting for delta orientation.
/// \return A workspace trajectory with unit velocity timing.
std::unique_ptr<aikido::trajectory::Interpolated>
timeTrajectoryByGeodesicUnitTiming(
    const aikido::trajectory::Interpolated* untimedTraj,
    const std::shared_ptr<aikido::statespace::SE3> SE3StateSpace,
    double alpha = 1.0);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
