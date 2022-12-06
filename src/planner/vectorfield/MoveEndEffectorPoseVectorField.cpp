#include "aikido/planner/vectorfield/MoveEndEffectorPoseVectorField.hpp"

#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>

#include "aikido/planner/vectorfield/VectorFieldUtil.hpp"

#include "detail/VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

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
  mNormalizedTwist = computeGeodesicTwist(mStartPose, mGoalPose);
  mNormalizedTwist.head<3>().normalize();
  mNormalizedTwist.tail<3>().normalize();
}

//==============================================================================
bool MoveEndEffectorPoseVectorField::evaluateCartesianVelocity(
    const Eigen::Isometry3d& /* pose */,
    Eigen::Vector6d& cartesianVelocity) const
{
  using aikido::planner::vectorfield::computeGeodesicTwist;
  cartesianVelocity = mDesiredTwist;
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

  if (poseError < mGoalTolerance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_TERMINATE;
  }

  // Check for deviation from the desired trajectory.
  const Eigen::Vector6d startTwist = computeGeodesicTwist(mStartPose, pose);
  const Eigen::Vector3d direction = mNormalizedTwist.tail<3>();
  const Eigen::Vector3d position = startTwist.tail<3>();
  double movedDistance = position.transpose() * direction;
  double positionDeviation = (position - movedDistance * direction).norm();

  const Eigen::Vector3d angle = startTwist.head<3>();
  const Eigen::Vector3d rotation = mNormalizedTwist.head<3>();
  double movedAngle = angle.transpose() * rotation;
  double orientationError = (angle - movedAngle * rotation).norm();

  if (fabs(orientationError) > mAngularTolerance)
  {
    dtwarn << "Deviated from orientation constraint: ("
           << fabs(orientationError) << " > " << mAngularTolerance << ")"
           << std::endl;
    return VectorFieldPlannerStatus::TERMINATE;
  }

  if (positionDeviation > mPositionTolerance)
  {
    dtwarn << "Deviated from straight-line constraint: "
           << fabs(positionDeviation) << " > " << mPositionTolerance << ")"
           << std::endl;
    return VectorFieldPlannerStatus::TERMINATE;
  }

  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
