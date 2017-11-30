#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
double MoveEndEffectorOffsetVectorField::InvalidMaxDistance
    = std::numeric_limits<double>::infinity();

//==============================================================================
MoveEndEffectorOffsetVectorField::MoveEndEffectorOffsetVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bn,
    const Eigen::Vector3d& direction,
    double distance,
    double maxDistance,
    double positionTolerance,
    double angularTolerance,
    double linearVelocityGain,
    double initialStepSize,
    double jointLimitPadding,
    double optimizationTolerance)
  : ConfigurationSpaceVectorField(stateSpace, bn)
  , mDirection(direction)
  , mDistance(distance)
  , mMaxDistance(maxDistance)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mLinearVelocityGain(linearVelocityGain)
  , mInitialStepSize(initialStepSize)
  , mJointLimitPadding(jointLimitPadding)
  , mOptimizationTolerance(optimizationTolerance)
  , mStartPose(bn->getTransform())
{
  if (mDistance < 0)
    throw std::invalid_argument("Distance must be non-negative.");
  if (mDirection.norm() == 0)
    throw std::invalid_argument("Direction must be non-zero");
  if (mMaxDistance < mDistance)
    throw std::invalid_argument("Max distance is less than minimum distance.");
  if (mPositionTolerance < 0)
    throw std::invalid_argument("Position tolerance is negative");
  if (mAngularTolerance < 0)
    throw std::invalid_argument("Angular tolerance is negative");
  if (mOptimizationTolerance < 0)
    throw std::invalid_argument("Optimization tolerance is negative");
}

//==============================================================================
bool MoveEndEffectorOffsetVectorField::getJointVelocities(
    Eigen::VectorXd& qd) const
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector6d;
  using dart::math::logMap;

  const Isometry3d currentPose = mBodyNode->getTransform();

  Eigen::VectorXd jointVelocityUpperLimits
      = mMetaSkeleton->getVelocityUpperLimits();
  Eigen::VectorXd jointVelocityLowerLimits
      = mMetaSkeleton->getVelocityLowerLimits();

  Vector6d desiredTwist = computeGeodesicTwist(currentPose, mStartPose);
  desiredTwist.tail<3>() = mDirection * mLinearVelocityGain;

  bool result = computeJointVelocityFromTwist(
      qd,
      desiredTwist,
      mStateSpace,
      mBodyNode,
      mJointLimitPadding,
      jointVelocityLowerLimits,
      jointVelocityUpperLimits,
      true,
      mInitialStepSize,
      mOptimizationTolerance);
  return result;
}

//==============================================================================
VectorFieldPlannerStatus MoveEndEffectorOffsetVectorField::checkPlanningStatus()
    const
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector4d;

  const Isometry3d currentPose = mBodyNode->getTransform();

  // Check for deviation from the straight-line trajectory.
  const Vector4d geodesicError = computeGeodesicError(mStartPose, currentPose);
  const double orientationError = geodesicError[0];
  const Vector3d positionError = geodesicError.tail<3>();
  double movedDistance = positionError.transpose() * mDirection;
  double positionDeviation
      = (positionError - movedDistance * mDirection).norm();

  if (fabs(orientationError) > mAngularTolerance)
  {
    throw VectorFieldError("Deviated from orientation constraint.");
  }

  if (positionDeviation > mPositionTolerance)
  {
    throw VectorFieldError("Deviated from straight line constraint.");
  }

  // Check if we've reached the target.
  if (mMaxDistance == InvalidMaxDistance)
  {
    if (movedDistance >= mDistance)
    {
      return VectorFieldPlannerStatus::CACHE_AND_TERMINATE;
    }
  }
  else if (movedDistance > mMaxDistance)
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  else if (movedDistance >= mDistance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_CONTINUE;
  }

  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
