#include "aikido/planner/dart/ConfigurationToEndEffectorPosePlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorPosePlanner::
    ConfigurationToEndEffectorPosePlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : dart::SingleProblemPlanner<ConfigurationToEndEffectorPosePlanner,
                               ConfigurationToEndEffectorPose>(
        std::move(stateSpace), std::move(metaSkeleton))
{
  // Do nothing
}

} // namespace dart
} // namespace planner
} // namespace aikido
