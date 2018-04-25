#include "aikido/planner/ConfigurationToTSR.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToTSR::ConfigurationToTSR(
    statespace::StateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    const statespace::StateSpace::State* startState,
    const constraint::dart::TSRPtr goalTSR,
    statespace::InterpolatorPtr interpolator,
    constraint::TestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mBodyNode(std::move(bodyNode))
  , mStartState(startState)
  , mGoalTSR(goalTSR)
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
const std::string& ConfigurationToTSR::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToTSR::getStaticType()
{
  static std::string name("ConfigurationToTSR");
  return name;
}

//==============================================================================
dart::dynamics::BodyNodePtr ConfigurationToTSR::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
const statespace::StateSpace::State* ConfigurationToTSR::getStartState() const
{
  return mStartState;
}

//==============================================================================
const constraint::dart::TSRPtr ConfigurationToTSR::getGoalTSR() const
{
  return mGoalTSR;
}

//==============================================================================
statespace::InterpolatorPtr ConfigurationToTSR::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::ConstInterpolatorPtr ConfigurationToTSR::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr ConfigurationToTSR::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToTSR::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
