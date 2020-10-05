#include "aikido/planner/dart/ConfigurationToConfigurations.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfigurations::ConfigurationToConfigurations(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
    const GoalStates& goalStates,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(std::move(metaSkeleton))
  , mStartState(mMetaSkeletonStateSpace->createState())
{
  for (const statespace::dart::MetaSkeletonStateSpace::State* goal : goalStates)
  {
    mGoalStates.push_back(stateSpace->cloneState(goal));
  }
}

//==============================================================================
ConfigurationToConfigurations::ConfigurationToConfigurations(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    const GoalStates& goalStates,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mMetaSkeletonStateSpace(stateSpace)
  , mMetaSkeleton(nullptr)
  , mStartState(stateSpace->cloneState(startState))
{
  for (const statespace::dart::MetaSkeletonStateSpace::State* goal : goalStates)
  {
    mGoalStates.push_back(stateSpace->cloneState(goal));
  }
}

//==============================================================================
const std::string& ConfigurationToConfigurations::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToConfigurations::getStaticType()
{
  static std::string name("ConfigurationToConfigurations");
  return name;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
ConfigurationToConfigurations::getStartState() const
{
  // Take start state from MetaSkeleton if passed. Store in the ScopedState
  // instance variable to avoid dangling pointers.
  if (mMetaSkeleton)
    mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), mStartState);

  return mStartState;
}

//==============================================================================
std::size_t ConfigurationToConfigurations::getNumGoalStates() const
{
  return mGoalStates.size();
}

//==============================================================================
ConfigurationToConfigurations::GoalStates
ConfigurationToConfigurations::getGoalStates() const
{
  std::vector<const statespace::dart::MetaSkeletonStateSpace::State*>
      pointerStates;
  for (const auto& goal : mGoalStates)
  {
    pointerStates.push_back(goal.getState());
  }
  return pointerStates;
}

} // namespace dart
} // namespace planner
} // namespace aikido
