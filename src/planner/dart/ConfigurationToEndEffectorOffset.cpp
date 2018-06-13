#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const statespace::dart::MetaSkeletonStateSpace::State* startState,
    const Eigen::Vector3d& direction,
    const double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mStartState(startState)
  , mDirection(direction.normalized())
  , mDistance(signedDistance)
{
  if (direction.squaredNorm() == 0)
    throw std::invalid_argument("direction shouldn't be a zero vector.");
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
::dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorOffset::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
const statespace::dart::MetaSkeletonStateSpace::State*
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

} // namespace dart
} // namespace planner
} // namespace aikido
