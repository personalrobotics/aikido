#include "aikido/planner/PlanToEndEffectorPose.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
PlanToEndEffectorPose::PlanToEndEffectorPose(
    statespace::ConstStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    const Eigen::Vector3d& direction,
    statespace::InterpolatorPtr interpolator,
    constraint::TestablePtr constraint,
    const Eigen::Isometry3d& goalPose)
  : Problem(std::move(stateSpace))
  , mBodyNode(std::move(bodyNode))
  , mStartState(startState)
  , mGoalState(goalState)
  , mDirection(direction)
  , mInterpolator(std::move(interpolator))
  , mConstraint(std::move(constraint))
  , mGoalPose(goalPose)
{
  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
}

//==============================================================================
const std::string& PlanToEndEffectorPose::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& PlanToEndEffectorPose::getStaticName()
{
  static std::string name("PlanToEndEffectorPose");
  return name;
}

//==============================================================================
dart::dynamics::BodyNodePtr PlanToEndEffectorPose::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
const statespace::StateSpace::State* PlanToEndEffectorPose::getStartState()
    const
{
  return mStartState;
}

//==============================================================================
const statespace::StateSpace::State* PlanToEndEffectorPose::getGoalState() const
{
  return mStartState;
}

//==============================================================================
const Eigen::Vector3d& PlanToEndEffectorPose::getDirection() const
{
  return mDirection;
}

//==============================================================================
statespace::InterpolatorPtr PlanToEndEffectorPose::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::ConstInterpolatorPtr PlanToEndEffectorPose::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr PlanToEndEffectorPose::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::ConstTestablePtr PlanToEndEffectorPose::getConstraint() const
{
  return mConstraint;
}

//==============================================================================
const Eigen::Isometry3d&PlanToEndEffectorPose::getGoalPose() const
{
  return mGoalPose;
}

} // namespace planner
} // namespace aikido
