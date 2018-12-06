#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"
//#include "aikido/planner/dart/util.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
//    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
//    const Eigen::Vector3d& direction,
    const double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(mMetaSkeletonStateSpace->createState())
//  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
//  , mDirection(direction)
  , mDistance(signedDistance)
{
//  if (mDirection.get().isZero())
//    throw std::invalid_argument("direction shouldn't be a zero vector.");
}

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
//    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
//    const Eigen::Vector3d& direction,
    double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
//  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
//  , mDirection(direction)
  , mDistance(signedDistance)
{
//  if (mDirection.get().isZero())
//    throw std::invalid_argument("direction shouldn't be a zero vector.");
}

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    const Eigen::Vector3d& direction,
    const double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(mMetaSkeletonStateSpace->createState())
  , mDirection(direction)
  , mDistance(signedDistance)
{
  if (mDirection.get().isZero())
    throw std::invalid_argument("direction shouldn't be a zero vector.");
}

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    const Eigen::Vector3d& direction,
    double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
  , mDirection(direction)
  , mDistance(signedDistance)
{
  if (mDirection.get().isZero())
    throw std::invalid_argument("direction shouldn't be a zero vector.");
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
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToEndEffectorOffset::getStartState() const
{
  // Take start state from MetaSkeleton if passed. Store in the ScopedState
  // instance variable to avoid dangling pointers.
  if (mMetaSkeleton)
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), mStartState);

  return mStartState;
}

//==============================================================================
boost::optional<Eigen::Vector3d> ConfigurationToEndEffectorOffset::getDirection() const
{
  boost::optional<Eigen::Vector3d> direction;

  if (mDirection)
  {
    direction = mDirection.get().normalized();
  }
  return direction;
}


//==============================================================================
double ConfigurationToEndEffectorOffset::getDistance() const
{
  return mDistance;
}

/*
//==============================================================================
std::shared_ptr<Problem> ConfigurationToEndEffectorOffset::clone(
  ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton) const
{
  // TODO: assert that metaSkeleton matches mMetaSkeleton

  auto clonedBodyNode = metaSkeleton->getBodyNode(
      mEndEffectorBodyNode->getName())->getBodyNodePtr();

  return std::make_shared<ConfigurationToEndEffectorOffset>(
      mMetaSkeletonStateSpace,
      metaSkeleton,
      clonedBodyNode,
      mDirection.get(),
      mDistance,
      mConstraint);
}
*/

} // namespace dart
} // namespace planner
} // namespace aikido
