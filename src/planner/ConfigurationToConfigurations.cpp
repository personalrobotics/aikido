#include "aikido/planner/ConfigurationToConfigurations.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurations::ConfigurationToConfigurations(
    statespace::StateSpacePtr stateSpace,
    const statespace::StateSpace::State* startState,
    const std::vector<statespace::StateSpace::State*>& goalStates,
    statespace::InterpolatorPtr interpolator,
    constraint::TestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mStartState(startState)
  , mGoalStates(goalStates)
  , mInterpolator(std::move(interpolator))
  , mConstraint(std::move(constraint))
{
  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
}

//==============================================================================
const std::string& ConfigurationToConfigurations::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& ConfigurationToConfigurations::getStaticName()
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
const std::vector<statespace::StateSpace::State*>
ConfigurationToConfigurations::getGoalStates() const
{
  return mGoalStates;
}

//==============================================================================
statespace::InterpolatorPtr ConfigurationToConfigurations::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::ConstInterpolatorPtr
ConfigurationToConfigurations::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr ConfigurationToConfigurations::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToConfigurations::getConstraint()
    const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
