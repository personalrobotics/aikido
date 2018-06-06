#include "aikido/planner/dart/ConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::
    ConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : dart::SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                               ConfigurationToEndEffectorOffset>(
        std::move(stateSpace), std::move(metaSkeleton))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
