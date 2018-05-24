#include "aikido/planner/dart/ConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::
    ConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                         ConfigurationToEndEffectorOffset>(stateSpace)
  , mMetaSkeletonStateSpace(std::move(stateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  // Do nothing
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr
ConfigurationToEndEffectorOffsetPlanner::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

} // namespace dart
} // namespace planner
} // namespace aikido
