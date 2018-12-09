#include "aikido/planner/dart/ConfigurationToEndEffectorOffsetPlanner.hpp"
#include "aikido/planner/dart/util.hpp"

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

/*
//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::
    ConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
        const Eigen::Vector3d& direction)
  : dart::SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                               ConfigurationToEndEffectorOffset>(
        std::move(stateSpace), std::move(metaSkeleton))
  , mDirection(direction)
{
  if (endEffectorBodyNode)
    setEndEffectorBodyNode(endEffectorBodyNode);

  if (mDirection.get().isZero()
      throw std::invalid_argument("direction shouldn't be a zero vector.");
}
*/

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

//==============================================================================
Eigen::Vector3d
ConfigurationToEndEffectorOffsetPlanner::getEndEffectorDirection() const
{
  return util::getEndEffectorDirection(mEndEffectorBodyNode);
}

//==============================================================================
bool ConfigurationToEndEffectorOffsetPlanner::stopPlanning()
{
  return false;
}

} // namespace dart
} // namespace planner
} // namespace aikido
