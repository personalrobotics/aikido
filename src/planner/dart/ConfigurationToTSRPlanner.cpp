#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSRPlanner::ConfigurationToTSRPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode)
  : dart::SingleProblemPlanner<ConfigurationToTSRPlanner, ConfigurationToTSR>(
        std::move(stateSpace), std::move(metaSkeleton))
{
  if (endEffectorBodyNode)
    setEndEffectorBodyNode(endEffectorBodyNode);
}

//==============================================================================
::dart::dynamics::ConstBodyNodePtr
ConfigurationToTSRPlanner::getEndEffectorBodyNode() const
{
  return mEndEffectorBodyNode;
}


//==============================================================================
void ConfigurationToTSRPlanner::setEndEffectorBodyNode(
  ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode)
{
  mEndEffectorBodyNode = std::move(endEffectorBodyNode);
}

} // namespace dart
} // namespace planner
} // namespace aikido
