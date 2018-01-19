#include "aikido/planner/PlanToConfigurations.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
PlanToConfigurations::PlanToConfigurations(
    statespace::ConstStateSpacePtr stateSpace,
    const statespace::StateSpace::State* startState,
    const std::vector<statespace::StateSpace::State*> goalStates,
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
const std::string& PlanToConfigurations::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& PlanToConfigurations::getStaticName()
{
  static std::string name("PlanToConfigurations");
  return name;
}

//==============================================================================
const statespace::StateSpace::State* PlanToConfigurations::getStartState() const
{
  return mStartState;
}

//==============================================================================
const std::vector<statespace::StateSpace::State*>
PlanToConfigurations::getGoalStates() const
{
  return mGoalStates;
}

//==============================================================================
statespace::InterpolatorPtr PlanToConfigurations::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::ConstInterpolatorPtr PlanToConfigurations::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr PlanToConfigurations::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::ConstTestablePtr PlanToConfigurations::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
