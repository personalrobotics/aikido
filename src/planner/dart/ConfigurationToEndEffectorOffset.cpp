#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffset::ConfigurationToEndEffectorOffset(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const boost::optional<Eigen::Vector3d>& direction,
    const double signedDistance,
    constraint::ConstTestablePtr constraint)
  : Problem(stateSpace, std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mDirection(direction)
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
const Eigen::Vector3d ConfigurationToEndEffectorOffset::getDirection() const
{
  if (!mDirection)
  {
    return mEndEffectorBodyNode->getWorldTransform()
        .linear()
        .col(2)
        .normalized();
  }

  return mDirection.get().normalized();
}

//==============================================================================
double ConfigurationToEndEffectorOffset::getDistance() const
{
  return mDistance;
}

} // namespace dart
} // namespace planner
} // namespace aikido
