#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

MoveEndEffectorOffsetVectorField::MoveEndEffectorOffsetVectorField(
    dart::dynamics::BodyNodePtr _bn,
    const Eigen::Vector3d& _linearVelocity,
    double _startTime,
    double _endTime,
    double _timestep,
    double _linearGain,
    double _linearTolerance,
    double _rotationGain,
    double _rotationTolerance,
    double _optimizationTolerance,
    double _padding)
  : mBodynode(std::move(_bn))
  , mVelocity(_linearVelocity)
  , mLinearDirection(_linearVelocity.normalized())
  , mStartTime(_startTime)
  , mEndTime(_endTime)
  , mTimestep(_timestep)
  , mLinearGain(_linearGain)
  , mLinearTolerance(_linearTolerance)
  , mAngularGain(_rotationGain)
  , mAngularTolerance(_rotationTolerance)
  , mOptimizationTolerance(_optimizationTolerance)
  , mPadding(_padding)
  , mStartPose(_bn->getTransform())
{
  if (mStartTime < 0.0)
      throw std::invalid_argument("Start time is negative");
  if (mEndTime < mStartTime)
      throw std::invalid_argument("End time is smaller than start time");
  if (mTimestep <= 0.0)
      throw std::invalid_argument("Time step is negative");

  if (mLinearGain < 0)
      throw std::invalid_argument("Linear gain is negative");
  if (mLinearTolerance <0)
      throw std::invalid_argument("Linear tolerance is negative");
  if (mAngularGain < 0)
      throw std::invalid_argument("Angular gain is negative");
  if (mAngularTolerance < 0)
      throw std::invalid_argument("Angular tolerance is negative");
  if (mOptimizationTolerance < 0)
      throw std::invalid_argument("Optimization tolerance is negative");

  mTargetPose = mStartPose;
  mTargetPose.translation() += mVelocity * (mEndTime - mStartTime);
  mEndTime += _padding;
}

//==============================================================================
bool MoveEndEffectorOffsetVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
    double,
    Eigen::VectorXd& _dq)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector6d;
  using dart::math::logMap;

  const Isometry3d currentPose = mBodynode->getTransform();
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
  // Compute rotational error.
  const Vector3d rotationError = logMap(mTargetPose.rotation().inverse() * currentPose.rotation());

  // Compute the desired twist using a proportional controller.
  Vector6d desiredTwist;
  desiredTwist.head<3>() = mAngularGain * rotationError;
  desiredTwist.tail<3>()
      = linearFeedforward + mLinearGain * linearOrthogonalError;

  bool result = computeJointVelocityFromTwist(
      desiredTwist,
      _stateSpace,
      mBodynode,
      mOptimizationTolerance,
      mTimestep,
      mPadding,
      _dq);
  return result;
}

//==============================================================================
VectorFieldPlannerStatus MoveEndEffectorOffsetVectorField::operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
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
  if (_t > mEndTime)
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  else if (_t >= mStartTime)
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
