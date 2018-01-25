#include "aikido/planner/PlanToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
PlanToEndEffectorOffset::PlanToEndEffectorOffset(
    statespace::StateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bodyNode,
    const statespace::StateSpace::State* startState,
    const Eigen::Vector3d& direction,
    const double& distance,
    statespace::InterpolatorPtr interpolator,
    constraint::TestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mBodyNode(std::move(bodyNode))
  , mStartState(startState)
  , mDirection(direction)
  , mDistance(distance)
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
const std::string& PlanToEndEffectorOffset::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& PlanToEndEffectorOffset::getStaticName()
{
  static std::string name("PlanToEndEffectorOffset");
  return name;
}

//==============================================================================
dart::dynamics::BodyNodePtr PlanToEndEffectorOffset::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
const statespace::StateSpace::State* PlanToEndEffectorOffset::getStartState()
    const
{
  return mStartState;
}

//==============================================================================
const Eigen::Vector3d& PlanToEndEffectorOffset::getDirection() const
{
  return mDirection;
}

//==============================================================================
const double& PlanToEndEffectorOffset::getDistance() const
{
  return mDistance;
}

//==============================================================================
statespace::InterpolatorPtr PlanToEndEffectorOffset::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::ConstInterpolatorPtr PlanToEndEffectorOffset::getInterpolator()
    const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr PlanToEndEffectorOffset::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::ConstTestablePtr PlanToEndEffectorOffset::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
