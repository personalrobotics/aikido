#include "aikido/planner/ConfigurationToConfigurations.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurations::ConfigurationToConfigurations(
    statespace::ConstStateSpacePtr stateSpace,
    const statespace::StateSpace::State* startState,
    const GoalStates& goalStates,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mStartState(stateSpace->cloneState(startState))
{
  for (const auto& goalState : goalStates)
    mGoalStates.emplace(stateSpace->cloneState(goalState));
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
const statespace::StateSpace::State*
ConfigurationToConfigurations::getStartState() const
{
  return mStartState;
}

//==============================================================================
std::size_t ConfigurationToConfigurations::getNumGoalStates() const
{
  return mGoalStates.size();
}

//==============================================================================
const ConfigurationToConfigurations::ScopedGoalStates&
ConfigurationToConfigurations::getGoalStates() const
{
  return mGoalStates;
}

} // namespace planner
} // namespace aikido
