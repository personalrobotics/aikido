#include "aikido/planner/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToTSR::
    ConfigurationToConfiguration_to_ConfigurationToTSR(
        std::shared_ptr<ConfigurationToConfigurationPlanner> planner)
  : PlannerAdapter<ConfigurationToConfigurationPlanner,
                   ConfigurationToTSRPlanner>(std::move(planner))
{
  // Do nothing
}

//==============================================================================
trajectory::TrajectoryPtr
ConfigurationToConfiguration_to_ConfigurationToTSR::plan(
    const ConfigurationToTSR& /*problem*/, Planner::Result* /*result*/)
{
  // TODO
  return nullptr;
}

} // namespace planner
} // namespace aikido
