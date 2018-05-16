#include "aikido/planner/ConfigurationToConfiguration.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfiguration::ConfigurationToConfiguration(
    statespace::ConstStateSpacePtr stateSpace,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mStartState(stateSpace->cloneState(startState))
  , mGoalState(stateSpace->cloneState(goalState))
{
  // Do nothing
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
const statespace::StateSpace::State*
ConfigurationToConfiguration::getStartState() const
{
  return mStartState;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToConfiguration::getGoalState() const
{
  return mGoalState;
}

} // namespace planner
} // namespace aikido
