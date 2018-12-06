#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSRParallel.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToTSRParallel::
    ConfigurationToConfiguration_to_ConfigurationToTSRParallel(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      const std::shared_ptr<ConfigurationToConfiguration_to_ConfigurationToTSR>&
          planner,
      std::size_t numCopies,
      const std::vector<common::RNG*> rngs)
: ConcreteParallelMetaPlanner(std::move(stateSpace), std::move(metaSkeleton),
    planner, numCopies, rngs)
{
}

} // namespace dart
} // namespace planner
} // namespace aikido
