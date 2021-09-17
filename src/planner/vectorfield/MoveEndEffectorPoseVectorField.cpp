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
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    dart::dynamics::MetaSkeletonPtr metaskeleton,
    dart::dynamics::BodyNodePtr bn,
    const Eigen::Isometry3d& goalPose,
    double poseErrorTolerance,
    double r,
    double maxStepSize,
    double jointLimitPadding)
  : BodyNodePoseVectorField(
      stateSpace, metaskeleton, bn, maxStepSize, jointLimitPadding)
  , mGoalPose(goalPose)
  , mPoseErrorTolerance(poseErrorTolerance)
  , mConversionRatioFromRadiusToMeter(r)
{
  if (mPoseErrorTolerance < 0)
    throw std::invalid_argument("Pose error tolerance is negative");
}

//==============================================================================
bool MoveEndEffectorPoseVectorField::evaluateCartesianVelocity(
    const Eigen::Isometry3d& pose, Eigen::Vector6d& cartesianVelocity) const
{
  using aikido::planner::vectorfield::computeGeodesicTwist;
  Eigen::Vector6d desiredTwist = computeGeodesicTwist(pose, mGoalPose);
  cartesianVelocity = desiredTwist;
  return true;
}

//==============================================================================
VectorFieldPlannerStatus
MoveEndEffectorPoseVectorField::evaluateCartesianStatus(
    const Eigen::Isometry3d& pose) const
{
  double poseError = computeGeodesicDistance(
      pose, mGoalPose, mConversionRatioFromRadiusToMeter);

  if (poseError < mPoseErrorTolerance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_TERMINATE;
  }
  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
