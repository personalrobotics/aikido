#include "aikido/planner/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::StateSpacePtr stateSpace,
    dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const statespace::StateSpace::State* startState,
    const Eigen::Vector3d& direction,
    double distance,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mStartState(startState)
  , mDirection(direction)
  , mDistance(distance)
  , mConstraint(std::move(constraint))
{
  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
}

//==============================================================================
const std::string& ConfigurationToEndEffectorOffset::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& ConfigurationToEndEffectorOffset::getStaticType()
{
  static std::string name("ConfigurationToEndEffectorOffset");
  return name;
}

//==============================================================================
void ConfigurationToEndEffectorOffset::setEndEffectorBodyNode(
    dart::dynamics::ConstBodyNodePtr endEffectorBodyNode)
{
  mEndEffectorBodyNode = std::move(endEffectorBodyNode);
}

//==============================================================================
dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorOffset::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
void ConfigurationToEndEffectorOffset::setStartState(
    const statespace::StateSpace::State* startState)
{
  mStartState = startState;
}

//==============================================================================
const statespace::StateSpace::State*
ConfigurationToEndEffectorOffset::getStartState() const
{
  return mStartState;
}

//==============================================================================
void ConfigurationToEndEffectorOffset::setDirection(
    const Eigen::Vector3d& direction)
{
  mDirection = direction.normalized();
}

//==============================================================================
const Eigen::Vector3d& ConfigurationToEndEffectorOffset::getDirection() const
{
  return mDirection;
}

//==============================================================================
void ConfigurationToEndEffectorOffset::setDistance(double distance)
{
  // TODO(JS): Check validity of distance.

  mDistance = distance;
}

//==============================================================================
double ConfigurationToEndEffectorOffset::getDistance() const
{
  return mDistance;
}

//==============================================================================
constraint::ConstTestablePtr ConfigurationToEndEffectorOffset::getConstraint()
    const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
