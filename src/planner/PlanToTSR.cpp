#include "aikido/planner/PlanToTSR.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
PlanToTSR::PlanToTSR(
    statespace::ConstStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    const statespace::StateSpace::State* startState,
    const constraint::TSRPtr goalTSR,
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
const std::string& PlanToTSR::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& PlanToTSR::getStaticName()
{
  static std::string name("PlanToTSR");
  return name;
}

//==============================================================================
dart::dynamics::BodyNodePtr PlanToTSR::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
const statespace::StateSpace::State* PlanToTSR::getStartState() const
{
  return mStartState;
}

//==============================================================================
const constraint::TSRPtr PlanToTSR::getGoalTSR() const
{
  return mGoalTSR;
}

//==============================================================================
statespace::InterpolatorPtr PlanToTSR::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::ConstInterpolatorPtr PlanToTSR::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr PlanToTSR::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::ConstTestablePtr PlanToTSR::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
