#include "aikido/planner/ConfigurationToTSRPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToTSRPlanner::ConfigurationToTSRPlanner(
    statespace::ConstStateSpacePtr stateSpace)
  : SingleProblemPlanner<ConfigurationToTSRPlanner, ConfigurationToTSR>(
        std::move(stateSpace))
{
  // Do nothing
}

} // namespace planner
} // namespace aikido
