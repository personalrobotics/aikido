#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurationPlanner::ConfigurationToConfigurationPlanner(
    statespace::ConstStateSpacePtr stateSpace, std::unique_ptr<common::RNG> rng)
  : SingleProblemPlanner<ConfigurationToConfigurationPlanner,
                         ConfigurationToConfiguration>(
        std::move(stateSpace), std::move(rng))
{
  // Do nothing
}

} // namespace planner
} // namespace aikido
