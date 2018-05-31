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
  : Problem(stateSpace, std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mStartState(std::move(stateSpace->cloneState(startState)))
  , mDirection(direction.normalized())
  , mDistance(signedDistance)
{
  // Do nothing.
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
const Eigen::Vector3d ConfigurationToEndEffectorOffset::getDirection() const
{
  if (mDirection.squaredNorm() == 0)
  {
    return mEndEffectorBodyNode->getWorldTransform()
        .linear()
        .col(2)
        .normalized();
  }

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
