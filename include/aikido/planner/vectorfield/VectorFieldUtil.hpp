#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <aikido/common/Spline.hpp>
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
/// \param[in] jointVelocityUpperLimits Joint velocity upper bounds.
/// \param[in] enforceJointVelocityLimits Whether joint velocity limits are
/// considered in computation.
/// \param[in] stepSize Step size in second. It is used in evaluating
/// position bounds violation. It assumes that whether moving the time of
/// stepSize by maximum joint velocity will reach the limit.
/// \return Whether a joint velocity is found
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& jointVelocity,
    const Eigen::Vector6d& desiredTwist,
    const dart::dynamics::MetaSkeletonPtr stateSpace,
    const dart::dynamics::BodyNodePtr bodyNode,
    double jointLimitPadding,
    const Eigen::VectorXd& jointVelocityLowerLimits,
    const Eigen::VectorXd& jointVelocityUpperLimits,
    bool enforceJointVelocityLimits,
    double stepSize);

/// Compute the twist in global coordinate that corresponds to the gradient of
/// the geodesic distance between two transforms.
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \return Geodesic twist in global coordinate. It corresponds to the gradient
/// of the geodesic distance between two transforms. The first three are angular
/// velocities (meters per second), and the last three are linear velocities
/// (radian per sec).
Eigen::Vector6d computeGeodesicTwist(
    const Eigen::Isometry3d& fromTrans, const Eigen::Isometry3d& toTrans);

/// Compute the error in gloabl coordinate between two transforms.
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \return Geodesic error in global coordinate. It is a 4d vector, in which
/// the first element is the norm of angle difference (in radian), and the
/// last three elements are translation difference (in meter).
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

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
