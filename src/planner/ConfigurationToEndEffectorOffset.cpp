#include "aikido/planner/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::ConstStateSpacePtr stateSpace,
    dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const statespace::StateSpace::State* startState,
    const Eigen::Vector3d& direction,
    const double distance,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mStartState(startState)
  , mDirection(direction.normalized())
  , mDistance(distance)
{
  if (direction.norm() < 1e-6)
    throw std::invalid_argument("Length of direction shouldn't be zero.");

  if (distance < 0.0)
    throw std::invalid_argument("Distance shouldn't be a negative value.");
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
dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorOffset::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
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
double ConfigurationToEndEffectorOffset::getDistance() const
{
  return mDistance;
}

} // namespace planner
} // namespace aikido
