#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/detail/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/detail/VectorFieldUtil.hpp>

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
    double jointLimitPadding)
  : BodyNodePoseVectorField(stateSpace, bn, initialStepSize, jointLimitPadding)
  , mDirection(direction)
  , mDistance(distance)
  , mMaxDistance(maxDistance)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mLinearVelocityGain(linearVelocityGain)
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

  // Normalize the direction vector
  mDirection.normalize();
}

//==============================================================================
bool MoveEndEffectorOffsetVectorField::evaluateCartesianVelocity(
    const Eigen::Isometry3d& pose, Eigen::Vector6d& cartesianVelocity) const
{
  using aikido::planner::vectorfield::computeGeodesicError;

  Eigen::Vector6d desiredTwist = computeGeodesicTwist(pose, mStartPose);
  desiredTwist.tail<3>() = mDirection * mLinearVelocityGain;
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
