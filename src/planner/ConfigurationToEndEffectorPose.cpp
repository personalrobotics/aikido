#include "aikido/planner/ConfigurationToEndEffectorPose.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorPose::ConfigurationToEndEffectorPose(
    statespace::StateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    const statespace::StateSpace::State* startState,
    const Eigen::Isometry3d& goalPose,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mBodyNode(std::move(bodyNode))
  , mStartState(startState)
  , mGoalPose(goalPose)
  , mConstraint(std::move(constraint))
{
  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
}

//==============================================================================
const std::string& ConfigurationToEndEffectorPose::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToEndEffectorPose::getStaticType()
{
  static std::string name("ConfigurationToEndEffectorPose");
  return name;
}

//==============================================================================
dart::dynamics::BodyNodePtr ConfigurationToEndEffectorPose::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
void ConfigurationToEndEffectorPose::setStartState(
    const statespace::StateSpace::State* startState)
{
  mStartState = startState;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToEndEffectorPose::getStartState() const
{
  return mStartState;
}

//==============================================================================
void ConfigurationToEndEffectorPose::setGoalPose(
    const Eigen::Isometry3d& goalPose)
{
  mGoalPose = goalPose;
}

//==============================================================================
const Eigen::Isometry3d& ConfigurationToEndEffectorPose::getGoalPose() const
{
  return mGoalPose;
}

//==============================================================================
void ConfigurationToEndEffectorPose::setConstraint(
    constraint::ConstTestablePtr constraint)
{
  mConstraint = std::move(constraint);
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToEndEffectorPose::getConstraint()
    const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
