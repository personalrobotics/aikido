#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurationPlanner::ConfigurationToConfigurationPlanner(
    statespace::ConstStateSpacePtr stateSpace)
  : SingleProblemPlanner<ConfigurationToConfigurationPlanner,
                         ConfigurationToConfiguration>(std::move(stateSpace))
{
  // Do nothing
}

} // namespace planner
} // namespace aikido
