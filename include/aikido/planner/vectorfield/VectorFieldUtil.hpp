#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

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
/// If input is nullptr, it means no joint velocity lower limits.
/// \param[in] jointVelocityUpperLimits Joint velocity upper bounds.
/// If input is nullptr, it means no joint velocity upper limits.
/// \param[in] stepSize Step size in second. It is used in evaluating
/// position bounds violation. It assumes that whether moving the time of
/// stepSize by maximum joint velocity will reach the limit.
/// \return Whether a joint velocity is found
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& jointVelocity,
    const Eigen::Vector6d& desiredTwist,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    double jointLimitPadding,
    const Eigen::VectorXd* jointVelocityLowerLimits,
    const Eigen::VectorXd* jointVelocityUpperLimits,
    double stepSize);

/// Compute the twist in global coordinate that corresponds to the gradient of
/// the geodesic distance between two transforms.
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \return Geodesic twist in global coordinate. The first three are angular
/// velocities, and the last three are linear velocities.
Eigen::Vector6d computeGeodesicTwist(
    const Eigen::Isometry3d& fromTrans, const Eigen::Isometry3d& toTrans);

/// Compute the error in gloabl coordinate between two transforms.
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \return Geodesic error in global coordinate. It is a 4d vector, in which
/// the first element is the norm of angle difference, and the last three
/// elements are translation difference.
Eigen::Vector4d computeGeodesicError(
    const Eigen::Isometry3d& fromTrans, const Eigen::Isometry3d& toTrans);

/// Compute the geodesic distance between two transforms.
/// gd = norm( relative translation + r * axis-angle error )
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \param[in] r In units of meters/radians converts radians to meters.
/// \return Geodesic distance in meter.
double computeGeodesicDistance(
    const Eigen::Isometry3d& fromTrans,
    const Eigen::Isometry3d& toTrans,
    double r);

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
