#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>

#include "aikido/planner/vectorfield/MoveEndEffectorTwistVectorField.hpp"
#include "aikido/planner/vectorfield/VectorFieldUtil.hpp"

#include "detail/VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
MoveEndEffectorTwistVectorField::MoveEndEffectorTwistVectorField(
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    ::dart::dynamics::ConstBodyNodePtr bn,
    const Eigen::Vector6d& twistSeq,
    double durationSeq,
    double positionTolerance,
    double angularTolerance,
    double maxStepSize,
    double jointLimitPadding)
  : BodyNodePoseVectorField(
        stateSpace, metaskeleton, bn, maxStepSize, jointLimitPadding)
  , mTwist(twistSeq)
  , mDuration(durationSeq)
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
  DART_UNUSED(pose);
  cartesianVelocity = mTwist;
  return true;
}

//==============================================================================
VectorFieldPlannerStatus
MoveEndEffectorTwistVectorField::evaluateCartesianStatus(
    const Eigen::Isometry3d& pose) const
{
  mCount += 1;
  DART_UNUSED(pose);

  // (avk): Not computing the error/deviation for now

  // (avk): if the action has been applied for given time, stop.
  double threshold = mDuration / mMaxStepSize;
  if (mCount > (int)threshold)
  {
    return VectorFieldPlannerStatus::CACHE_AND_TERMINATE;
  }

  return VectorFieldPlannerStatus::CACHE_AND_CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido