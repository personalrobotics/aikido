#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfigurationPlanner::ConfigurationToConfigurationPlanner(
    const statespace::ConstStateSpacePtr& stateSpace)
  : PlannerForSingleProblem<ConfigurationToConfigurationPlanner,
                            ConfigurationToConfiguration>(std::move(stateSpace))
{
  // Do nothing
}

} // namespace planner
} // namespace aikido
