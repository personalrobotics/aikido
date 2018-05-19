#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_

#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ConfigurationToTSRPlanner.hpp"
#include "aikido/planner/PlannerAdapter.hpp"

namespace aikido {
namespace planner {

class ConfigurationToConfiguration_to_ConfigurationToTSR
    : public PlannerAdapter<ConfigurationToConfigurationPlanner,
                            ConfigurationToTSRPlanner>
{
public:
  ConfigurationToConfiguration_to_ConfigurationToTSR(
      std::shared_ptr<ConfigurationToConfigurationPlanner> planner);

  trajectory::TrajectoryPtr plan(
      const ConfigurationToTSR& problem, Planner::Result* result) override;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_
