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

//==============================================================================
std::shared_ptr<Planner> ConfigurationToEndEffectorOffsetPlanner::clone() const
{
  throw std::runtime_error("Not implemented");
}

} // namespace dart
} // namespace planner
} // namespace aikido
