#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSRPlanner::ConfigurationToTSRPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : SingleProblemPlanner<ConfigurationToTSRPlanner, ConfigurationToTSR>(
        stateSpace)
  , mMetaSkeletonStateSpace(std::move(stateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  // Do nothing
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr
ConfigurationToTSRPlanner::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

} // namespace dart
} // namespace planner
} // namespace aikido
