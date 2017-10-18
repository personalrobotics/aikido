#include "MoveHandStraightVectorField.hpp"
#include <fstream>
#include <iostream>
#include <boost/format.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include "VectorFieldUtil.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

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
  , timestep_(stepsize)
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
  assert(timestep_ >= 0);
  assert(velocity_.all() >= 0);
  assert(linear_gain_ >= 0);
  assert(linear_tolerance_ > 0);
  assert(rotation_gain_ >= 0);
  assert(rotation_tolerance_ > 0);
  assert(optimization_tolerance_ > 0);

  target_pose_ = start_pose_;
  target_pose_.translation() += velocity_ * max_duration_;
  max_duration_ += timestep_ / 20;
}

bool MoveHandStraightVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t,
    Eigen::VectorXd* qd)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::VectorXd;

  DART_UNUSED(t);
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  Isometry3d const current_pose = bodynode_->getTransform();

  // Compute the "feed-forward" term necessary to move at a constant velocity
  // from the start to the goal.
  Vector3d const linear_direction = velocity_.normalized();
  Vector3d const& linear_feedforward = velocity_;

  // Compute positional error orthogonal to the direction of motion by
  // projecting out the component of the error along that direction.
  Vector3d const linear_error
      = target_pose_.translation() - current_pose.translation();
  Vector3d const linear_orthogonal_error
      = linear_error - linear_error.dot(linear_direction) * linear_direction;

  // Compute rotational error.
  Vector3d const rotation_error = dart::math::logMap(
      target_pose_.rotation().inverse() * current_pose.rotation());

  // Compute the desired twist using a proportional controller.
  Vector6d desired_twist;
  desired_twist.head<3>() = rotation_gain_ * rotation_error;
  desired_twist.tail<3>()
      = linear_feedforward + linear_gain_ * linear_orthogonal_error;

  bool result = ComputeJointVelocityFromTwist(
      desired_twist,
      stateSpace,
      bodynode_,
      optimization_tolerance_,
      timestep_,
      padding_,
      qd);
  return result;
}

VectorFieldPlannerStatus::Enum MoveHandStraightVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t)
{
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
