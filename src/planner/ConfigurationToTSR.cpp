#include "aikido/planner/ConfigurationToTSR.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToTSR::ConfigurationToTSR(
    statespace::StateSpacePtr stateSpace,
    dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const statespace::StateSpace::State* startState,
    constraint::dart::ConstTSRPtr goalTSR,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mStartState(startState)
  , mGoalTSR(goalTSR)
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
void ConfigurationToTSR::setEndEffectorBodyNode(
    dart::dynamics::ConstBodyNodePtr endEffectorBodyNode)
{
  mEndEffectorBodyNode = std::move(endEffectorBodyNode);
}

//==============================================================================
dart::dynamics::ConstBodyNodePtr ConfigurationToTSR::getEndEffectorBodyNode()
    const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
void ConfigurationToTSR::setStartState(
    const statespace::StateSpace::State* startState)
{
  mStartState = startState;
}

//==============================================================================
const statespace::StateSpace::State* ConfigurationToTSR::getStartState() const
{
  return mStartState;
}

//==============================================================================
void ConfigurationToTSR::setGoalTSR(constraint::dart::ConstTSRPtr goalTSR)
{
  mGoalTSR = std::move(goalTSR);
}

//==============================================================================
constraint::dart::ConstTSRPtr ConfigurationToTSR::getGoalTSR() const
{
  return mGoalTSR;
}

//==============================================================================
void ConfigurationToTSR::setConstraint(constraint::ConstTestablePtr constraint)
{
  mConstraint = std::move(constraint);
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToTSR::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
