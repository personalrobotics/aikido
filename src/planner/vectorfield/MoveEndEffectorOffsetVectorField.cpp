#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>

#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>

#include "detail/VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
MoveEndEffectorOffsetVectorField::MoveEndEffectorOffsetVectorField(
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    ::dart::dynamics::ConstBodyNodePtr bn,
    const Eigen::Vector3d& direction,
    double minDistance,
    double maxDistance,
    double positionTolerance,
    double angularTolerance,
    double maxStepSize,
    double jointLimitPadding)
  : BodyNodePoseVectorField(
        stateSpace, metaskeleton, bn, maxStepSize, jointLimitPadding)
  , mDirection(direction)
  , mMinDistance(minDistance)
  , mMaxDistance(maxDistance)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mStartPose(bn->getTransform())
{
  if (mMinDistance < 0)
    throw std::invalid_argument("Minimum distance must be non-negative.");
  if (mDirection.norm() == 0)
    throw std::invalid_argument("Direction must be non-zero");
  if (mMaxDistance < mMinDistance)
    throw std::invalid_argument("Max distance is less than minimum distance.");
  if (mPositionTolerance < 0)
    throw std::invalid_argument("Position tolerance is negative");
  if (mAngularTolerance < 0)
    throw std::invalid_argument("Angular tolerance is negative");

  // Normalize the direction vector
  mDirection.normalize();
}

//==============================================================================
bool MoveEndEffectorOffsetVectorField::evaluateCartesianVelocity(
    const Eigen::Isometry3d& pose, Eigen::Vector6d& cartesianVelocity) const
{
  using aikido::planner::vectorfield::computeGeodesicError;
  using ::dart::math::logMap;

  Eigen::Vector6d desiredTwist = computeGeodesicTwist(pose, mStartPose);
  desiredTwist.tail<3>() = mDirection;
  cartesianVelocity = desiredTwist;
  return true;
}

//==============================================================================
VectorFieldPlannerStatus
MoveEndEffectorOffsetVectorField::evaluateCartesianStatus(
    const Eigen::Isometry3d& pose) const
{
  using aikido::planner::vectorfield::computeGeodesicError;

  // Check for deviation from the straight-line trajectory.
  const Eigen::Vector4d geodesicError = computeGeodesicError(mStartPose, pose);
  const double orientationError = geodesicError[0];
  const Eigen::Vector3d positionError = geodesicError.tail<3>();
  double movedDistance = positionError.transpose() * mDirection;
  double positionDeviation
      = (positionError - movedDistance * mDirection).norm();

  if (fabs(orientationError) > mAngularTolerance)
  {
    dtwarn << "Deviated from orientation constraint.";
    return VectorFieldPlannerStatus::TERMINATE;
  }

  if (positionDeviation > mPositionTolerance)
  {
    dtwarn << "Deviated from straight line constraint.";
    return VectorFieldPlannerStatus::TERMINATE;
  }

  // if larger than max distance, terminate
  // if larger than min distance, cache and continue
  // if smaller than min distance, continue
  if (movedDistance > mMaxDistance)
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  else if (movedDistance >= mMinDistance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_CONTINUE;
  }

  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
