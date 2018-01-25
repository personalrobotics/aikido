#include "aikido/planner/PlanToConfiguration.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
PlanToConfiguration::PlanToConfiguration(
    statespace::StateSpacePtr stateSpace,
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
const std::string& PlanToConfiguration::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& PlanToConfiguration::getStaticName()
{
  static std::string name("PlanToConfiguration");
  return name;
}

//==============================================================================
const statespace::StateSpace::State* PlanToConfiguration::getStartState() const
{
  return mStartState;
}

//==============================================================================
const statespace::StateSpace::State* PlanToConfiguration::getGoalState() const
{
  return mGoalState;
}

//==============================================================================
statespace::InterpolatorPtr PlanToConfiguration::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::InterpolatorPtr PlanToConfiguration::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr PlanToConfiguration::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::TestablePtr PlanToConfiguration::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
