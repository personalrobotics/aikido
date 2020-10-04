#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONS_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONS_HPP_

#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToConfigurationsPlanner.hpp"
#include "aikido/planner/dart/PlannerAdapter.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Converts a non-DART ConfigurationToConfiguration planner into a DART
/// ConfigurationToTSR planner.
class ConfigurationToConfiguration_to_ConfigurationToConfigurations
  : public PlannerAdapter<
        aikido::planner::ConfigurationToConfigurationPlanner,
        ConfigurationToConfigurationsPlanner>
{
public:
  /// Constructor
  ///
  /// \param[in] planner Non-DART ConfigurationToConfigurationPlanner planner to
  /// convert.
  /// \param[in] metaSkeleton MetaSkeleton for adapted planner to operate on.
  /// \param[in] configurationRanker Ranker to rank the sampled configurations. If nullptr,
  /// NominalConfigurationRanker is used with the current metaSkeleton pose.
  ConfigurationToConfiguration_to_ConfigurationToTSR(
      std::shared_ptr<aikido::planner::ConfigurationToConfigurationPlanner>
          planner,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      distance::ConstConfigurationRankerPtr configurationRanker = nullptr);

  // Documentation inherited.
  virtual trajectory::TrajectoryPtr plan(
      const ConfigurationToTSR& problem, Planner::Result* result) override;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONS_HPP_
