#include "aikido/planner/dart/ConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfigurationPlanner::ConfigurationToConfigurationPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : SingleProblemPlanner<ConfigurationToConfigurationPlanner,
                         ConfigurationToConfiguration>(stateSpace)
  , mMetaSkeletonStateSpace(std::move(stateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  // Do nothing
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr
ConfigurationToConfigurationPlanner::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
::dart::dynamics::MetaSkeletonPtr
ConfigurationToConfigurationPlanner::getMetaSkeleton()
{
  return mMetaSkeleton;
}

} // namespace dart
} // namespace planner
} // namespace aikido
