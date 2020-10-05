#include "aikido/planner/dart/ConfigurationToConfigurationsPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfigurationsPlanner::ConfigurationToConfigurationsPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    distance::ConstConfigurationRankerPtr configurationRanker)
  : dart::SingleProblemPlanner<
        ConfigurationToConfigurationsPlanner,
        ConfigurationToConfigurations>(
        std::move(stateSpace), std::move(metaSkeleton))
  , mConfigurationRanker(std::move(configurationRanker))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
