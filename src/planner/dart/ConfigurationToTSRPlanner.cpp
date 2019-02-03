#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSRPlanner::ConfigurationToTSRPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    distance::ConfigurationRankerPtr configurationRanker)
  : dart::SingleProblemPlanner<ConfigurationToTSRPlanner, ConfigurationToTSR>(
        std::move(stateSpace), std::move(metaSkeleton))
  , mConfigurationRanker(std::move(configurationRanker))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
