#include "aikido/planner/dart/ConfigurationToEndEffectorPose.hpp"

#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/dart/DartConstraint.hpp"

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

//==============================================================================
std::shared_ptr<Problem> ConfigurationToEndEffectorPose::clone() const
{
  throw std::runtime_error(
      "ConfigurationToEndEffectorPose: clone with metaSkeleton should be used.");
}

//==============================================================================
std::shared_ptr<Problem> ConfigurationToEndEffectorPose::clone(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
  ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  using aikido::constraint::dart::DartConstraint;
  auto constraint = std::dynamic_pointer_cast<const DartConstraint>(mConstraint);

  // TODO: assert that metaSkeleton matches mMetaSkeleton
  auto clonedBodyNode = metaSkeleton->getBodyNode(0)->getSkeleton()->getBodyNode(
      mEndEffectorBodyNode->getName())->getBodyNodePtr();

  if (!clonedBodyNode)
  {
    std::stringstream ss;
    ss << "Metaskeleton does not have "
      << mEndEffectorBodyNode->getName() << std::endl;
    throw std::invalid_argument(ss.str());
  }

  if (!constraint)
    return std::make_shared<ConfigurationToEndEffectorPose>(
      mMetaSkeletonStateSpace, mStartState,
      clonedBodyNode,
      mGoalPose, mConstraint);

  else{
    std::cout << "ConfigurationToEndEffectorPose: Cloning mConstraint with "
              << "metaSkeleton\n";
    auto clonedConstraint = constraint->clone(collisionDetector, metaSkeleton);
    return std::make_shared<ConfigurationToEndEffectorPose>(
      mMetaSkeletonStateSpace, mStartState,
      clonedBodyNode, mGoalPose, clonedConstraint);
  }
}


} // namespace dart
} // namespace planner
} // namespace aikido
