#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSRPARALLEL_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSRPARALLEL_HPP_

#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"
#include "aikido/planner/dart/ConcreteParallelMetaPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Converts a non-DART ConfigurationToConfiguration planner into a DART
/// ConfigurationToTSR planner.
class ConfigurationToConfiguration_to_ConfigurationToTSRParallel
    : public ConcreteParallelMetaPlanner
{
public:
  /// Constructor
  ///
  /// \param[in] stateSpace StateSpace in which the planner operates.
  /// \param[in] metaSkeleton MetaSkeleton for adapted planner to operate on.
  /// \param[in] collisionDetector CollisionDetector used by all problems.
  /// \param[in] planner Planner to parallelize
  /// TSR.
  ConfigurationToConfiguration_to_ConfigurationToTSRParallel(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      ::dart::collision::CollisionDetectorPtr collisionDetector,
      const std::shared_ptr<ConfigurationToConfiguration_to_ConfigurationToTSR>&
          planner,
      std::size_t numCopies = 1,
      const std::vector<common::RNG*> rngs =
        std::vector<common::RNG*>());

};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONTOCONFIGURATIONTOTSR_HPP_
