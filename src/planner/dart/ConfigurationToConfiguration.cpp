#include "aikido/planner/dart/ConfigurationToConfiguration.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration::ConfigurationToConfiguration(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    const statespace::dart::MetaSkeletonStateSpace::State* goalState,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(mMetaSkeletonStateSpace->createState())
  , mGoalState(stateSpace->cloneState(goalState))
{
  // Do nothing.
}

//==============================================================================
ConfigurationToConfiguration::ConfigurationToConfiguration(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    const statespace::dart::MetaSkeletonStateSpace::State* goalState,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
  , mGoalState(stateSpace->cloneState(goalState))
{
  // Do nothing.
}

//==============================================================================
const std::string& ConfigurationToConfiguration::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToConfiguration::getStaticType()
{
  static std::string name("ConfigurationToConfiguration");
  return name;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToConfiguration::getStartState() const
{
  // Take start state from MetaSkeleton if passed. Store in the ScopedState
  // instance variable to avoid dangling pointers.
  if (mMetaSkeleton)
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), mStartState);

  return mStartState;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToConfiguration::getGoalState() const
{
  return mGoalState;
}

} // namespace dart
} // namespace planner
} // namespace aikido
