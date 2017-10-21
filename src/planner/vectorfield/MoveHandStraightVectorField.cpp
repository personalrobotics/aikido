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
    dart::dynamics::BodyNodePtr _bn,
    const Eigen::Vector3d& _linearVelocity,
    double _minDuration,
    double _maxDuration,
    double _stepsize,
    double _linearGain,
    double _linearTolerance,
    double _rotationGain,
    double _rotationTolerance,
    double _optimizationTolerance,
    double _padding)
  : mBodynode(_bn)
  , mVelocity(_linearVelocity)
  , mLinearDirection(_linearVelocity.normalized())
  , mMinDuration(_minDuration)
  , mMaxDuration(_maxDuration)
  , mTimestep(_stepsize)
  , mLinearGain(_linearGain)
  , mLinearTolerance(_linearTolerance)
  , mRotationGain(_rotationGain)
  , mRotationTolerance(_rotationTolerance)
  , mOptimizationTolerance(_optimizationTolerance)
  , mPadding(_padding)
  , mStartPose(_bn->getTransform())
{
  assert(mMinDuration >= 0);
  assert(mMaxDuration >= mMinDuration);
  assert(mTimestep >= 0);
  assert(mVelocity.all() >= 0);
  assert(mLinearGain >= 0);
  assert(mLinearTolerance > 0);
  assert(mRotationGain >= 0);
  assert(mRotationTolerance > 0);
  assert(mOptimizationTolerance > 0);

  mTargetPose = mStartPose;
  mTargetPose.translation() += mVelocity * mMaxDuration;
  mMaxDuration += _padding;
}

//==============================================================================

bool MoveHandStraightVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    double _t,
    Eigen::VectorXd* _qd)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::VectorXd;
  using Eigen::Vector6d;

  DART_UNUSED(_t);

  Isometry3d const currentPose = mBodynode->getTransform();
  Vector3d currentWorkspacePose = currentPose.translation();
  Vector3d targetWorkspacePose = mTargetPose.translation();

  // Compute the "feed-forward" term necessary to move at a constant velocity
  // from the start to the goal.
  const Vector3d& linearFeedforward = mVelocity;

  // Compute positional error orthogonal to the direction of motion by
  // projecting out the component of the error along that direction.
  const Vector3d linearError = targetWorkspacePose - currentWorkspacePose;
  const Vector3d linearOrthogonalError
      = linearError - linearError.dot(mLinearDirection) * mLinearDirection;

  // Compute the desired twist using a proportional controller.
  Vector6d desiredTwist;
  desiredTwist.head<3>() = Eigen::Vector3d::Zero();
  desiredTwist.tail<3>()
      = linearFeedforward + mLinearGain * linearOrthogonalError;

  bool result = computeJointVelocityFromTwist(
      desiredTwist,
      _stateSpace,
      mBodynode,
      mOptimizationTolerance,
      mTimestep,
      mPadding,
      _qd);
  return result;
}

//==============================================================================

VectorFieldPlannerStatus MoveHandStraightVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    double _t)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;

  DART_UNUSED(_stateSpace);
  const Isometry3d currentPose = mBodynode->getTransform();

  // Check for deviation from the straight-line trajectory.
  const Vector3d linearError
      = mTargetPose.translation() - currentPose.translation();
  const Vector3d linearOrthogonalError
      = linearError - linearError.dot(mLinearDirection) * mLinearDirection;
  double linearOrthogonalMagnitude = linearOrthogonalError.norm();

  if (linearOrthogonalMagnitude >= mLinearTolerance)
  {
    dtwarn << "Trajectory deviated from the straight line by "
           << linearOrthogonalMagnitude << " m; the tolerance is "
           << mLinearTolerance << " m.\n";
    return VectorFieldPlannerStatus::TERMINATE;
  }

  // Check if we've reached the target.
  if (_t > mMaxDuration)
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  else if (_t >= mMinDuration)
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
