#include "aikido/planner/ConfigurationToConfigurations.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurations::ConfigurationToConfigurations(
    statespace::StateSpacePtr stateSpace,
    const statespace::StateSpace::State* startState,
    const GoalStates& goalStates,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mStartState(startState)
  , mGoalStates(goalStates)
  , mConstraint(std::move(constraint))
{
  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
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
void ConfigurationToConfigurations::setStartState(
    statespace::StateSpace::State* startState)
{
  mStartState = startState;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToConfigurations::getStartState() const
{
  return mStartState;
}

//==============================================================================
void ConfigurationToConfigurations::setGoalStates(
    const ConfigurationToConfigurations::GoalStates& goalStates)
{
  mGoalStates = goalStates;
}

//==============================================================================
void ConfigurationToConfigurations::addGoalState(
    const statespace::StateSpace::State* goalState)
{
  // TODO(JS): Do we want to report the success?
  mGoalStates.insert(goalState);
}

//==============================================================================
const ConfigurationToConfigurations::GoalStates&
ConfigurationToConfigurations::getGoalStates() const
{
  return mGoalStates;
}

//==============================================================================
void ConfigurationToConfigurations::setConstraint(
    constraint::ConstTestablePtr constraint)
{
  mConstraint = std::move(constraint);
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToConfigurations::getConstraint()
    const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
