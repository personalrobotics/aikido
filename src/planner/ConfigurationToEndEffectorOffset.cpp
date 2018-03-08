#include "aikido/planner/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
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
const std::string& ConfigurationToEndEffectorOffset::getName() const
{
  return getStaticName();
}

//==============================================================================
const std::string& ConfigurationToEndEffectorOffset::getStaticName()
{
  static std::string name("ConfigurationToEndEffectorOffset");
  return name;
}

//==============================================================================
dart::dynamics::BodyNodePtr ConfigurationToEndEffectorOffset::getBodyNode()
{
  return mBodyNode;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToEndEffectorOffset::getStartState() const
{
  return mStartState;
}

//==============================================================================
const Eigen::Vector3d& ConfigurationToEndEffectorOffset::getDirection() const
{
  return mDirection;
}

//==============================================================================
const double& ConfigurationToEndEffectorOffset::getDistance() const
{
  return mDistance;
}

//==============================================================================
statespace::InterpolatorPtr ConfigurationToEndEffectorOffset::getInterpolator()
{
  return mInterpolator;
}

//==============================================================================
statespace::ConstInterpolatorPtr
ConfigurationToEndEffectorOffset::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::TestablePtr ConfigurationToEndEffectorOffset::getConstraint()
{
  return mConstraint;
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToEndEffectorOffset::getConstraint()
    const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
