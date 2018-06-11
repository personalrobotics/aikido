#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"

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
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(nullptr)
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
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
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
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(nullptr)
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
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
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
const Eigen::Vector3d ConfigurationToEndEffectorOffset::getDirection() const
{
  if (!mDirection)
  {
    return mEndEffectorBodyNode->getWorldTransform()
        .linear()
        .col(2)
        .normalized();
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
