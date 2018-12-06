#include "aikido/planner/dart/ConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::
    ConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode)
  : dart::SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                               ConfigurationToEndEffectorOffset>(
        std::move(stateSpace), std::move(metaSkeleton))
{
  if (endEffectorBodyNode)
    setEndEffectorBodyNode(endEffectorBodyNode);
}

//==============================================================================
::dart::dynamics::ConstBodyNodePtr
ConfigurationToEndEffectorOffsetPlanner::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}

//==============================================================================
void ConfigurationToEndEffectorOffsetPlanner::setEndEffectorBodyNode(
  ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode)
{
  mEndEffectorBodyNode = std::move(endEffectorBodyNode);
}

} // namespace dart
} // namespace planner
} // namespace aikido
