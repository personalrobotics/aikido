#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Solver.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include <aikido/common/Spline.hpp>
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

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
