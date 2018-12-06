#include "aikido/planner/dart/ConfigurationToEndEffectorPose.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorPose::ConfigurationToEndEffectorPose(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const Eigen::Isometry3d& goalPose,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(mMetaSkeletonStateSpace->createState())
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mGoalPose(goalPose)
{
  // Do nothing
}

//==============================================================================
ConfigurationToEndEffectorPose::ConfigurationToEndEffectorPose(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const Eigen::Isometry3d& goalPose,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mGoalPose(goalPose)
{
  // Do nothing
}

//==============================================================================
const std::string& ConfigurationToEndEffectorPose::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToEndEffectorPose::getStaticType()
{
  static std::string name("ConfigurationToEndEffectorPose");
  return name;
}

//==============================================================================
::dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorPose::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToEndEffectorPose::getStartState() const
{
  // Take start state from MetaSkeleton if passed. Store in the ScopedState
  // instance variable to avoid dangling pointers.
  if (mMetaSkeleton)
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), mStartState);

  return mStartState;
}

//==============================================================================
const Eigen::Isometry3d& ConfigurationToEndEffectorPose::getGoalPose() const
{
  return mGoalPose;
}

/*
//==============================================================================
std::shared_ptr<Problem> ConfigurationToEndEffectorPose::clone(
  ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton) const
{
  // TODO: assert that metaSkeleton matches mMetaSkeleton

  auto clonedBodyNode = metaSkeleton->getBodyNode(
      mEndEffectorBodyNode->getName())->getBodyNodePtr();

  if (!clonedBodyNode)
  {
    std::stringstream ss;
    ss << "Metaskeleton does not have "
      << mEndEffectorBodyNode->getName() << std::endl;
    throw std::invalid_argument(ss.str());
  }
  return std::make_shared<ConfigurationToEndEffectorPose>
    (mMetaSkeletonStateSpace, metaSkeleton,
     clonedBodyNode,
     mGoalPose, mConstraint);
}
*/

} // namespace dart
} // namespace planner
} // namespace aikido
