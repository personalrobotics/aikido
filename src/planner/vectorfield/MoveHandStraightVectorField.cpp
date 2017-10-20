#include "MoveHandStraightVectorField.hpp"
#include <fstream>
#include <iostream>
#include <boost/format.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

MoveHandStraightVectorField::MoveHandStraightVectorField(
    dart::dynamics::BodyNodePtr bn,
    Eigen::Vector3d const& linearVelocity,
    double minDuration,
    double maxDuration,
    double stepsize,
    double linearGain,
    double linearTolerance,
    double rotationGain,
    double rotationTolerance,
    double optimizationTolerance,
    double padding)
  : bodynode_(bn)
  , velocity_(linearVelocity)
  , linearDirection_(linearVelocity.normalized())
  , minDuration_(minDuration)
  , maxDuration_(maxDuration)
  , timestep_(stepsize)
  , linearGain_(linearGain)
  , linearTolerance_(linearTolerance)
  , rotationGain_(rotationGain)
  , rotationTolerance_(rotationTolerance)
  , optimizationTolerance_(optimizationTolerance)
  , padding_(padding)
  , startPose_(bn->getTransform())
{
  assert(minDuration_ >= 0);
  assert(maxDuration_ >= minDuration_);
  assert(timestep_ >= 0);
  assert(velocity_.all() >= 0);
  assert(linearGain_ >= 0);
  assert(linearTolerance_ > 0);
  assert(rotationGain_ >= 0);
  assert(rotationTolerance_ > 0);
  assert(optimizationTolerance_ > 0);

  targetPose_ = startPose_;
  targetPose_.translation() += velocity_ * maxDuration_;
  maxDuration_ += padding;
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
  Vector3d current_workspace_pose = current_pose.translation();
  Vector3d target_workspace_pose = targetPose_.translation();

  // Compute the "feed-forward" term necessary to move at a constant velocity
  // from the start to the goal.
  Vector3d const& linear_feedforward = velocity_;

  // Compute positional error orthogonal to the direction of motion by
  // projecting out the component of the error along that direction.
  Vector3d const linear_error = target_workspace_pose - current_workspace_pose;
  Vector3d const linear_orthogonal_error
      = linear_error - linear_error.dot(linearDirection_) * linearDirection_;

  // Compute the desired twist using a proportional controller.
  Vector6d desired_twist;
  desired_twist.head<3>()
      = Eigen::Vector3d::Zero(); // rotationGain_ * rotation_error;
  desired_twist.tail<3>()
      = linear_feedforward + linearGain_ * linear_orthogonal_error;

  bool result = ComputeJointVelocityFromTwist(
      desired_twist,
      stateSpace,
      bodynode_,
      optimizationTolerance_,
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
  Vector3d const linear_error
      = targetPose_.translation() - current_pose.translation();
  Vector3d const linear_orthogonal_error
      = linear_error - linear_error.dot(linearDirection_) * linearDirection_;
  double const linear_orthogonal_magnitude = linear_orthogonal_error.norm();

  if (linear_orthogonal_magnitude >= linearTolerance_)
  {
    dtwarn << "Trajectory deviated from the straight line by "
           << linear_orthogonal_magnitude << " m; the tolerance is "
           << linearTolerance_ << " m.\n";
    return VectorFieldPlannerStatus::TERMINATE;
  }

  // Check if we've reached the target.
  if (t > maxDuration_)
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  else if (t >= minDuration_)
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
