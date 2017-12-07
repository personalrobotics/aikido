#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Solver.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/planner/vectorfield/VectorFieldIntegrator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

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
  /// \param[out] grad Gradient of a defined objective.
  void evalGradient(
      const Eigen::VectorXd& qd, Eigen::Map<Eigen::VectorXd> grad) override;

private:
  /// Twist.
  Twist mTwist;

  /// Jacobian of Meta Skeleton.
  Jacobian mJacobian;
};

/// Convert a sequence of waypoint and time pairs into a trajectory.
///
/// \param[in] A seqeunce of waypoint and time pairs.
/// \param[in] stateSpace State space of output trajectory.
/// \return A trajectory.
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    const aikido::statespace::StateSpacePtr stateSpace);

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
/// \param[in] stepSize Step size in second.
/// \return Whether a joint velocity is found
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& jointVelocity,
    const Eigen::Vector6d& desiredTwist,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    double jointLimitPadding,
    const Eigen::VectorXd& jointVelocityLowerLimits,
    const Eigen::VectorXd& jointVelocityUpperLimits,
    bool jointVelocityLimited,
    double stepSize);

/// Compute the twist in global coordinate that corresponds to the gradient of
/// the geodesic distance between two transforms.
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \return Geodesic twist.
Eigen::Vector6d computeGeodesicTwist(
    const Eigen::Isometry3d& fromTrans, const Eigen::Isometry3d& toTrans);

/// Compute the error in gloabl coordinate between two transforms.
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \return Geodesic error.
Eigen::Vector4d computeGeodesicError(
    const Eigen::Isometry3d& fromTrans, const Eigen::Isometry3d& toTrans);

/// Compute the geodesic distance between two transforms.
/// gd = norm( relative translation + r * axis-angle error )
///
/// \param[in] fromTrans Current transformation.
/// \param[in] toTrans Goal transformation.
/// \param[in] r In units of meters/radians converts radians to meters.
/// \return Geodesic distance.
double computeGeodesicDistance(
    const Eigen::Isometry3d& fromTrans,
    const Eigen::Isometry3d& toTrans,
    double r);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
