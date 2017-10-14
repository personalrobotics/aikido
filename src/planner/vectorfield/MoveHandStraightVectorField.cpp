#include "MoveHandStraightVectorField.hpp"
#include <fstream>
#include <iostream>
#include <boost/format.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

class DesiredTwistFunction : public dart::optimizer::Function
{
public:
  using Twist = Eigen::Matrix<double, 6, 1>;
  using Jacobian = dart::math::Jacobian;

  DesiredTwistFunction(const Twist& _twist, const Jacobian& _jacobian)
    : dart::optimizer::Function("DesiredTwistFunction")
    , mTwist(_twist)
    , mJacobian(_jacobian)
  {
  }

  double eval(const Eigen::VectorXd& _qd) override
  {
    return 0.5 * (mJacobian * _qd - mTwist).squaredNorm();
  }

  void evalGradient(
      const Eigen::VectorXd& _qd, Eigen::Map<Eigen::VectorXd> _grad) override
  {
    _grad = mJacobian.transpose() * (mJacobian * _qd - mTwist);
  }

private:
  Twist mTwist;
  Jacobian mJacobian;
};

MoveHandStraightVectorField::MoveHandStraightVectorField(
    dart::dynamics::BodyNodePtr bn,
    Eigen::Vector3d const& linear_velocity,
    double min_duration,
    double max_duration,
    double stepsize,
    double linear_gain,
    double linear_tolerance,
    double rotation_gain,
    double rotation_tolerance,
    double optimization_tolerance,
    double padding)
  : bodynode_(bn)
  , velocity_(linear_velocity)
  , min_duration_(min_duration)
  , max_duration_(max_duration)
  , timsetep_(stepsize)
  , linear_gain_(linear_gain)
  , linear_tolerance_(linear_tolerance)
  , rotation_gain_(rotation_gain)
  , rotation_tolerance_(rotation_tolerance)
  , optimization_tolerance_(optimization_tolerance)
  , padding_(padding)
  , start_pose_(bn->getTransform())
{
  assert(min_duration_ >= 0);
  assert(max_duration_ >= min_duration_);
  assert(timsetep_ >= 0);
  assert(velocity_.all() >= 0);
  assert(linear_gain_ >= 0);
  assert(linear_tolerance_ > 0);
  assert(rotation_gain_ >= 0);
  assert(rotation_tolerance_ > 0);
  assert(optimization_tolerance_ > 0);

  target_pose_ = start_pose_;
  target_pose_.translation() += velocity_ * max_duration_;

  // consider linear tolerance in terminating by time
  double time_padding = linear_tolerance / linear_velocity.norm();
  max_duration_ += time_padding;
}

bool MoveHandStraightVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t,
    Eigen::VectorXd* qd)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::VectorXd;
  using Eigen::JacobiSVD;
  using dart::optimizer::Problem;
  using dart::math::Jacobian;
  using dart::math::logMap;

  DART_UNUSED(t);
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  Isometry3d const current_pose = bodynode_->getTransform();

  // Compute the "feed-forward" term necessary to move at a constant velocity
  // from the start to the goal.
  Vector3d const linear_direction = velocity_ / velocity_.norm();
  Vector3d const& linear_feedforward = velocity_;

  // Compute positional error orthogonal to the direction of motion by
  // projecting out the component of the error along that direction.
  Vector3d const linear_error
      = target_pose_.translation() - current_pose.translation();
  Vector3d const linear_orthogonal_error
      = linear_error - linear_error.dot(linear_direction) * linear_direction;

  // Compute rotational error.
  Vector3d const rotation_error
      = logMap(target_pose_.rotation().inverse() * current_pose.rotation());

  // Compute the desired twist using a proportional controller.
  Vector6d desired_twist;
  desired_twist.head<3>() = rotation_gain_ * rotation_error;
  desired_twist.tail<3>()
      = linear_feedforward + linear_gain_ * linear_orthogonal_error;

  // Use LBFGS to find joint angles that won't violate the joint limits.
  const Jacobian jacobian
      = stateSpace->getMetaSkeleton()->getWorldJacobian(bodynode_);
  const size_t numDofs = stateSpace->getMetaSkeleton()->getNumDofs();
  VectorXd lowerLimits(numDofs);
  VectorXd upperLimits(numDofs);

  for (size_t i = 0; i < numDofs; ++i)
  {
    const double position = stateSpace->getMetaSkeleton()->getPosition(i);
    const double positionLowerLimit
        = stateSpace->getMetaSkeleton()->getPositionLowerLimit(i);
    const double positionUpperLimit
        = stateSpace->getMetaSkeleton()->getPositionUpperLimit(i);
    const double velocityLowerLimit
        = stateSpace->getMetaSkeleton()->getVelocityLowerLimit(i);
    const double velocityUpperLimit
        = stateSpace->getMetaSkeleton()->getVelocityUpperLimit(i);

    if (position
        < positionLowerLimit - timsetep_ * velocityLowerLimit + padding_)
      lowerLimits[i] = 0;
    else
      lowerLimits[i] = velocityLowerLimit;

    if (position
        > positionUpperLimit - timsetep_ * velocityUpperLimit - padding_)
      upperLimits[i] = 0;
    else
      upperLimits[i] = velocityUpperLimit;
  }

  const auto problem = std::make_shared<Problem>(numDofs);
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);
  problem->setObjective(
      std::make_shared<DesiredTwistFunction>(desired_twist, jacobian));

  dart::optimizer::NloptSolver solver(problem, nlopt::LD_LBFGS);
  if (!solver.solve() || problem->getOptimumValue() > optimization_tolerance_)
  {
    return false;
  }

  *qd = problem->getOptimalSolution();
  return true;
}

VectorFieldPlannerStatus::Enum MoveHandStraightVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t)
{
  using boost::format;
  using boost::str;
  using Eigen::Isometry3d;
  using Eigen::Vector3d;

  DART_UNUSED(stateSpace);
  Isometry3d const current_pose = bodynode_->getTransform();

  // Check for deviation from the straight-line trajectory.
  Vector3d const linear_direction
      = (target_pose_.translation() - start_pose_.translation()).normalized();
  Vector3d const linear_error
      = target_pose_.translation() - current_pose.translation();
  Vector3d const linear_orthogonal_error
      = linear_error - linear_error.dot(linear_direction) * linear_direction;
  double const linear_orthogonal_magnitude = linear_orthogonal_error.norm();

  if (linear_orthogonal_magnitude >= linear_tolerance_)
  {
    dtwarn << "Trajectory deviated from the straight line by "
           << linear_orthogonal_magnitude << " m; the tolerance is "
           << linear_tolerance_ << " m.\n";
    return VectorFieldPlannerStatus::TERMINATE;
  }

  // Check if we've reached the target.
  if (t > max_duration_)
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  else if (t >= min_duration_)
  {
    return VectorFieldPlannerStatus::CACHE_AND_CONTINUE;
  }
  else
  {
    return VectorFieldPlannerStatus::CONTINUE;
  }
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
