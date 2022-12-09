#include "aikido/planner/vectorfield/MoveEndEffectorPoseVectorField.hpp"

#include <cmath>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>

#include "aikido/planner/vectorfield/VectorFieldUtil.hpp"

#include "detail/VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

// Define whether vector is "near zero"
#define NEARZERO 1E-8

//==============================================================================
MoveEndEffectorPoseVectorField::MoveEndEffectorPoseVectorField(
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    ::dart::dynamics::ConstBodyNodePtr bn,
    const Eigen::Isometry3d& goalPose,
    double goalTolerance,
    double r,
    double positionTolerance,
    double angularTolerance,
    double maxStepSize,
    double jointLimitPadding)
  : BodyNodePoseVectorField(
      stateSpace, metaskeleton, bn, maxStepSize, jointLimitPadding)
  , mGoalPose(goalPose)
  , mGoalTolerance(goalTolerance)
  , mConversionRatioFromRadiusToMeter(r)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mStartPose(bn->getTransform())
{
  if (mGoalTolerance < 0)
    throw std::invalid_argument("Goal tolerance is negative");
  if (mPositionTolerance < 0)
    throw std::invalid_argument("Position tolerance is negative");
  if (mAngularTolerance < 0)
    throw std::invalid_argument("Angular tolerance is negative");

  mDesiredTwist = computeGeodesicTwist(mStartPose, mGoalPose);
  mDirection = mDesiredTwist.tail<3>();
  mRotation = mDesiredTwist.head<3>();
  if (mDirection.norm() > NEARZERO)
    mDirection.normalize();
  else
    mDirection = Eigen::Vector3d::Zero();
  if (mRotation.norm() > NEARZERO)
    mRotation.normalize();
  else
    mRotation = Eigen::Vector3d::Zero();

  mDesiredTwist.normalize();

  mLastPoseError = std::make_shared<double>(computeGeodesicDistance(
      mStartPose, mGoalPose, mConversionRatioFromRadiusToMeter));
}

//==============================================================================
bool MoveEndEffectorPoseVectorField::evaluateCartesianVelocity(
    const Eigen::Isometry3d& /* pose */,
    Eigen::Vector6d& cartesianVelocity) const
{
  // Speed up large movements
  double scale = std::max(*mLastPoseError, 1.0);
  cartesianVelocity = scale * mDesiredTwist;
  return true;
}

//==============================================================================
VectorFieldPlannerStatus
MoveEndEffectorPoseVectorField::evaluateCartesianStatus(
    const Eigen::Isometry3d& pose) const
{

  // Check arrival at goal
  double poseError = computeGeodesicDistance(
      pose, mGoalPose, mConversionRatioFromRadiusToMeter);

  // Check for deviation from the desired trajectory.
  const Eigen::Vector6d startTwist = computeGeodesicTwist(mStartPose, pose);
  const Eigen::Vector3d position = startTwist.tail<3>();
  double movedDistance = position.transpose() * mDirection;
  double positionDeviation = (position - movedDistance * mDirection).norm();

  const Eigen::Vector3d angle = startTwist.head<3>();
  double movedAngle = angle.transpose() * mRotation;
  double orientationError = (angle - movedAngle * mRotation).norm();

  if (fabs(orientationError) > mAngularTolerance && movedAngle > NEARZERO)
  {
    dtwarn << "Deviated from orientation constraint: ("
           << fabs(orientationError) << " > " << mAngularTolerance << ")"
           << std::endl;
    return VectorFieldPlannerStatus::TERMINATE;
  }

  if (positionDeviation > mPositionTolerance && movedDistance > NEARZERO)
  {
    dtwarn << "Deviated from straight-line constraint: "
           << fabs(positionDeviation) << " > " << mPositionTolerance << ")"
           << std::endl;
    return VectorFieldPlannerStatus::TERMINATE;
  }

  // Cache if within goal range
  if (poseError < mGoalTolerance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_CONTINUE;
  }

  // End if error increases
  if (poseError > (*mLastPoseError))
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  *mLastPoseError = poseError;

  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
