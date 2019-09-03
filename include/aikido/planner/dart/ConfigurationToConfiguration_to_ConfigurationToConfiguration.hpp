#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOCONFIGURATION_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOCONFIGURATION_HPP_

#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/PlannerAdapter.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Converts a non-DART ConfigurationToConfiguration planner into the DART
/// version.
class ConfigurationToConfiguration_to_ConfigurationToConfiguration
  : public PlannerAdapter<
        planner::ConfigurationToConfigurationPlanner,
        planner::dart::ConfigurationToConfigurationPlanner>
{
public:
  /// Constructor
  ///
  /// \param[in] planner Non-DART planner to convert.
  /// \param[in] metaSkeleton MetaSkeleton for adapted planner to operate on.
  ConfigurationToConfiguration_to_ConfigurationToConfiguration(
      std::shared_ptr<planner::ConfigurationToConfigurationPlanner> planner,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton);

  // Documentation inherited.
  virtual trajectory::TrajectoryPtr plan(
      const planner::dart::ConfigurationToConfiguration& problem,
      Planner::Result* result) override;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOCONFIGURATION_HPP_
