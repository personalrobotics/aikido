#include "aikido/planner/dart/ConfigurationToEndEffectorPose.hpp"

#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorPose::ConfigurationToEndEffectorPose(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
    const Eigen::Isometry3d& goalPose,
    constraint::ConstTestablePtr constraint)
  : Problem(std::move(stateSpace), std::move(constraint))
  , mEndEffectorBodyNode(std::move(endEffectorBodyNode))
  , mGoalPose(goalPose)
{
  // Do nothing
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
::dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorPose::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
const Eigen::Isometry3d& ConfigurationToEndEffectorPose::getGoalPose() const
{
  return mGoalPose;
}

} // namespace dart
} // namespace planner
} // namespace aikido
