#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_

#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"
#include "aikido/planner/dart/PlannerAdapter.hpp"

namespace aikido {
namespace planner {
namespace dart {

class ConfigurationToConfiguration_to_ConfigurationToTSR
    : public PlannerAdapter<aikido::planner::
                                ConfigurationToConfigurationPlanner,
                            ConfigurationToTSRPlanner>
{
public:
  ConfigurationToConfiguration_to_ConfigurationToTSR(
      std::shared_ptr<aikido::planner::ConfigurationToConfigurationPlanner>
          planner,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      std::unique_ptr<common::RNG> rng);

  virtual trajectory::TrajectoryPtr plan(
      const ConfigurationToTSR& problem, Planner::Result* result) override;

protected:
  std::unique_ptr<common::RNG> mRng;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_
