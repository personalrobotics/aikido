#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_

#include "aikido/planner/dart/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"
#include "aikido/planner/dart/PlannerAdapter.hpp"

namespace aikido {
namespace planner {
namespace dart {

class ConfigurationToConfiguration_to_ConfigurationToTSR
    : public PlannerAdapter<ConfigurationToConfigurationPlanner,
                            ConfigurationToTSRPlanner>
{
public:
  ConfigurationToConfiguration_to_ConfigurationToTSR(
      std::shared_ptr<ConfigurationToConfigurationPlanner> planner);

  virtual trajectory::TrajectoryPtr plan(
      const ConfigurationToTSR& problem, Planner::Result* result) override;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_
