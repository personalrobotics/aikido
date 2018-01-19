#include "aikido/planner/PlanToEndEffectorPose.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
PlanToEndEffectorPose::PlanToEndEffectorPose(
    statespace::ConstStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    const statespace::StateSpace::State* startState,
    const Eigen::Isometry3d& goalPose,
    statespace::InterpolatorPtr interpolator,
    constraint::TestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mBodyNode(std::move(bodyNode))
  , mStartState(startState)
  , mGoalPose(goalPose)
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
const Eigen::Isometry3d& PlanToEndEffectorPose::getGoalPose() const
{
  return mGoalPose;
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

} // namespace planner
} // namespace aikido
