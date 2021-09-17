#include "aikido/planner/ConfigurationToConfigurationsPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurationsPlanner::ConfigurationToConfigurationsPlanner(
    statespace::ConstStateSpacePtr stateSpace, common::RNG* rng)
  : SingleProblemPlanner<
      ConfigurationToConfigurationsPlanner,
      ConfigurationToConfigurations>(std::move(stateSpace), std::move(rng))
{
  // Do nothing
}

} // namespace planner
} // namespace aikido
