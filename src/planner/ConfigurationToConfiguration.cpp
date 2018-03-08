#include "aikido/planner/ConfigurationToConfiguration.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfiguration::ConfigurationToConfiguration(
    const statespace::ConstStateSpacePtr& stateSpace,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    statespace::InterpolatorPtr interpolator,
    constraint::TestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mStartState(startState)
  , mGoalState(goalState)
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

//==============================================================================
statespace::InterpolatorPtr ConfigurationToConfiguration::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::InterpolatorPtr ConfigurationToConfiguration::getInterpolator()
    const
{
  return mInterpolator;
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
