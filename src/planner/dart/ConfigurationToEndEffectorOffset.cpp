#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/dart/util.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const boost::optional<Eigen::Vector3d>& direction,
    const double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(std::move(mMetaSkeletonStateSpace->createState()))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mDirection(direction)
  , mDistance(signedDistance)
{
  if (mDirection.get().squaredNorm() == 0)
    throw std::invalid_argument("direction shouldn't be a zero vector.");
}

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const boost::optional<Eigen::Vector3d>& direction,
    double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(std::move(stateSpace->cloneState(startState)))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mDirection(direction)
  , mDistance(signedDistance)
{
  if (mDirection.get().squaredNorm() == 0)
    throw std::invalid_argument("direction shouldn't be a zero vector.");
}

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(std::move(mMetaSkeletonStateSpace->createState()))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mDirection(boost::none)
  , mDistance(signedDistance)
{
  // Do nothing.
}

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(std::move(stateSpace->cloneState(startState)))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mDirection(boost::none)
  , mDistance(signedDistance)
{
  // Do nothing.
}

//==============================================================================
const std::string& ConfigurationToEndEffectorOffset::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToEndEffectorOffset::getStaticType()
{
  static std::string name("ConfigurationToEndEffectorOffset");
  return name;
}

//==============================================================================
::dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorOffset::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToEndEffectorOffset::getStartState() const
{
  // Take start state from MetaSkeleton if passed. Store in the ScopedState
  // instance variable to avoid dangling pointers.
  if (mMetaSkeleton)
  {
    auto startState = mMetaSkeletonStateSpace->createState();
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), startState);
    mStateSpace->copyState(startState, mStartState);
  }

  return mStartState;
}

//==============================================================================
const Eigen::Vector3d ConfigurationToEndEffectorOffset::getDirection() const
{
  if (!mDirection)
  {
    return util::getEndEffectorDirection(mEndEffectorBodyNode);
  }

  return mDirection.get().normalized();
}

//==============================================================================
double ConfigurationToEndEffectorOffset::getDistance() const
{
  return mDistance;
}

} // namespace dart
} // namespace planner
} // namespace aikido
