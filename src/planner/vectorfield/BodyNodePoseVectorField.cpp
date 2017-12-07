#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/planner/vectorfield/BodyNodePoseVectorField.hpp>
#include <aikido/planner/vectorfield/detail/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/detail/VectorFieldUtil.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
BodyNodePoseVectorField::BodyNodePoseVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    double initialStepSize,
    double jointLimitPadding)
  : VectorField(metaSkeletonStateSpace)
  , mMetaSkeletonStateSpace(metaSkeletonStateSpace)
  , mMetaSkeleton(metaSkeletonStateSpace->getMetaSkeleton())
  , mBodyNode(bodyNode)
  , mInitialStepSize(initialStepSize)
  , mJointLimitPadding(jointLimitPadding)
{
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
      mMetaSkeletonStateSpace,
      mBodyNode,
      mJointLimitPadding,
      mVelocityLowerLimits,
      mVelocityUpperLimits,
      true,
      mInitialStepSize);
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
    aikido::constraint::TestablePtr constraint,
    double evalStepSize) const
{
  using aikido::constraint::createTestableBounds;
  auto boundConstraint = createTestableBounds(mMetaSkeletonStateSpace);
  for (double t = trajectory.getStartTime(); t <= trajectory.getEndTime();
       t += evalStepSize)
  {
    auto state = mMetaSkeletonStateSpace->createState();
    trajectory.evaluate(t, state);

    // firstly check the bound
    if (!boundConstraint->isSatisfied(state))
    {
      throw DofLimitError();
    }

    // last check collision free constraint
    if (!constraint->isSatisfied(state))
    {
      throw ConstraintViolatedError();
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
