#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorTwistVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include "detail/VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
MoveEndEffectorTwistVectorField::MoveEndEffectorTwistVectorField(
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    ::dart::dynamics::ConstBodyNodePtr bn,
    const std::vector<Eigen::Vector6d>& twistSeq,
    const std::vector<double> durationSeq,
    double positionTolerance,
    double angularTolerance,
    double maxStepSize,
    double jointLimitPadding)
  : BodyNodePoseVectorField(
        stateSpace, metaskeleton, bn, maxStepSize, jointLimitPadding)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mStartPose(bn->getTransform())
{
  if (mPositionTolerance < 0)
    throw std::invalid_argument("Position tolerance is negative");
  if (mAngularTolerance < 0)
    throw std::invalid_argument("Angular tolerance is negative");
}

//==============================================================================
bool MoveEndEffectorTwistVectorField::evaluateCartesianVelocity(
    const Eigen::Isometry3d& pose, Eigen::Vector6d& cartesianVelocity) const
{
  using ::dart::math::logMap;
  using aikido::planner::vectorfield::computeGeodesicError;

  Eigen::Vector6d desiredTwist = computeGeodesicTwist(pose, mStartPose);
  //desiredTwist.tail<3>() = mDirection;
  cartesianVelocity = desiredTwist;
  return true;
}

//==============================================================================
VectorFieldPlannerStatus
MoveEndEffectorTwistVectorField::evaluateCartesianStatus(
    const Eigen::Isometry3d& pose) const
{
  using aikido::planner::vectorfield::computeGeodesicError;

  /*
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
  */

  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
