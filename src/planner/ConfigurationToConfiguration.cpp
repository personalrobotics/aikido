#include "aikido/planner/ConfigurationToConfiguration.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfiguration::ConfigurationToConfiguration(
    const statespace::ConstStateSpacePtr& stateSpace,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    constraint::TestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mStartState(startState)
  , mGoalState(goalState)
  , mConstraint(std::move(constraint))
{
  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
}

//==============================================================================
const std::string& ConfigurationToConfiguration::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& ConfigurationToConfiguration::getStaticName()
{
  static std::string name("ConfigurationToConfiguration");
  return name;
}

//==============================================================================
void ConfigurationToConfiguration::setStartState(
    const statespace::StateSpace::State* state)
{
  mStartState = state;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToConfiguration::getStartState() const
{
  return mStartState;
}

//==============================================================================
void ConfigurationToConfiguration::setGoalState(
    const statespace::StateSpace::State* state)
{
  mGoalState = state;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToConfiguration::getGoalState() const
{
  return mGoalState;
}

//==============================================================================
constraint::TestablePtr ConfigurationToConfiguration::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::TestablePtr ConfigurationToConfiguration::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
