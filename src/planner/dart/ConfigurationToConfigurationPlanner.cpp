#include "aikido/planner/dart/ConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfigurationPlanner::
    ConfigurationToConfigurationPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : dart::SingleProblemPlanner<ConfigurationToConfigurationPlanner,
                               ConfigurationToConfiguration>(
        std::move(stateSpace), std::move(metaSkeleton))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
