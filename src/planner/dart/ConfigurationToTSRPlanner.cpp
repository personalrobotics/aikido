#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToTSRPlanner::ConfigurationToTSRPlanner(
    statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : DartSingleProblemPlanner<ConfigurationToTSRPlanner, ConfigurationToTSR>(
        std::move(stateSpace), std::move(metaSkeleton))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
