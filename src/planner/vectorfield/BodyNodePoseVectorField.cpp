#include <aikido/common/StepSequence.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/planner/vectorfield/BodyNodePoseVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include "detail/VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
BodyNodePoseVectorField::BodyNodePoseVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    dart::dynamics::MetaSkeletonPtr metaSkeleton,
    dart::dynamics::BodyNodePtr bodyNode,
    double maxStepSize,
    double jointLimitPadding,
    bool enforceJointVelocityLimits)
  : VectorField(metaSkeletonStateSpace)
  , mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
  , mBodyNode(bodyNode)
  , mMaxStepSize(maxStepSize)
  , mJointLimitPadding(jointLimitPadding)
  , mEnforceJointVelocityLimits(enforceJointVelocityLimits)
{
  if (mMaxStepSize <= 0)
  {
    throw std::invalid_argument("Maximum step size must be non-negative.");
  }
  if (mJointLimitPadding <= 0)
  {
    throw std::invalid_argument("Joint limit padding must be non-negative.");
  }

  mVelocityLowerLimits = mMetaSkeleton->getVelocityLowerLimits();
  mVelocityUpperLimits = mMetaSkeleton->getVelocityUpperLimits();
}

//==============================================================================
bool BodyNodePoseVectorField::evaluateVelocity(
    const aikido::statespace::StateSpace::State* state,
    Eigen::VectorXd& qd) const
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector6d;
  using aikido::planner::vectorfield::computeJointVelocityFromTwist;

  Eigen::VectorXd position(mMetaSkeleton->getNumDofs());
  auto newState
      = static_cast<const aikido::statespace::CartesianProduct::State*>(state);
  mMetaSkeletonStateSpace->convertStateToPositions(newState, position);
  mMetaSkeleton->setPositions(position);

  const Isometry3d currentPose = mBodyNode->getTransform();

  Vector6d desiredTwist = Eigen::Vector6d::Zero();
  if (!evaluateCartesianVelocity(currentPose, desiredTwist))
  {
    return false;
  }

  bool result = computeJointVelocityFromTwist(
      qd,
      desiredTwist,
      mMetaSkeleton,
      mBodyNode,
      mJointLimitPadding,
      mVelocityLowerLimits,
      mVelocityUpperLimits,
      mEnforceJointVelocityLimits,
      mMaxStepSize);
  return result;
}

//==============================================================================
VectorFieldPlannerStatus BodyNodePoseVectorField::evaluateStatus(
    const aikido::statespace::StateSpace::State* state) const
{
  using Eigen::Isometry3d;

  Eigen::VectorXd position(mMetaSkeleton->getNumDofs());
  auto newState = static_cast<const aikido::statespace::dart::
                                  MetaSkeletonStateSpace::State*>(state);
  mMetaSkeletonStateSpace->convertStateToPositions(newState, position);
  mMetaSkeleton->setPositions(position);

  const Isometry3d currentPose = mBodyNode->getTransform();

  return evaluateCartesianStatus(currentPose);
}

//==============================================================================
bool BodyNodePoseVectorField::evaluateTrajectory(
    const aikido::trajectory::Trajectory& trajectory,
    const aikido::constraint::Testable* constraint,
    double evalStepSize,
    double& evalTimePivot,
    bool excludeEndTime) const
{
  if (constraint == nullptr)
  {
    return true;
  }
  auto state = mMetaSkeletonStateSpace->createState();

  aikido::common::StepSequence seq(
      evalStepSize,
      true,
      excludeEndTime,
      evalTimePivot,
      trajectory.getEndTime());

  for (double t : seq)
  {
    trajectory.evaluate(t, state);
    // update current evaluation time pivot
    evalTimePivot = t;
    if (!constraint->isSatisfied(state))
    {
      return false;
    }
  }

  return true;
}

//==============================================================================
aikido::statespace::dart::MetaSkeletonStateSpacePtr
BodyNodePoseVectorField::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
BodyNodePoseVectorField::getMetaSkeletonStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr BodyNodePoseVectorField::getMetaSkeleton()
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr BodyNodePoseVectorField::getMetaSkeleton()
    const
{
  return mMetaSkeleton;
}

//==============================================================================
dart::dynamics::BodyNodePtr BodyNodePoseVectorField::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
dart::dynamics::ConstBodyNodePtr BodyNodePoseVectorField::getBodyNode() const
{
  return mBodyNode;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
