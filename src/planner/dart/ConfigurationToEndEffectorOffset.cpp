#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/dart/DartConstraint.hpp"
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

//==============================================================================
std::shared_ptr<Problem> ConfigurationToEndEffectorOffset::clone() const
{
  // TODO: assert that metaSkeleton matches mMetaSkeleton
  throw std::runtime_error(
      "ConfigurationToEndEffectorOffset must call clone with MetaSkeleton");
}

//==============================================================================
std::shared_ptr<Problem> ConfigurationToEndEffectorOffset::clone(
  ::dart::collision::CollisionDetectorPtr collisionDetector,
  ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  using aikido::constraint::dart::DartConstraint;

  auto constraint = std::dynamic_pointer_cast<const DartConstraint>(mConstraint);

  if (!constraint)
  {
    if (mDirection)
    {
      return std::make_shared<ConfigurationToEndEffectorOffset>(
        mMetaSkeletonStateSpace,
        mStartState,
        mDirection.get(),
        mDistance,
        mConstraint);
    }
    return std::make_shared<ConfigurationToEndEffectorOffset>(
        mMetaSkeletonStateSpace,
        mStartState,
        mDistance,
        mConstraint);
  }
  else
  {
    auto clonedConstraint = constraint->clone(collisionDetector, metaSkeleton);
    if (mDirection)
    {
      return std::make_shared<ConfigurationToEndEffectorOffset>(
        mMetaSkeletonStateSpace,
        mStartState,
        mDirection.get(),
        mDistance,
        clonedConstraint);
    }
    return std::make_shared<ConfigurationToEndEffectorOffset>(
        mMetaSkeletonStateSpace,
        mStartState,
        mDistance,
        clonedConstraint);
  }
}



} // namespace dart
} // namespace planner
} // namespace aikido
