#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONSPLANNER_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONSPLANNER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"
#include "aikido/planner/dart/ConfigurationToConfigurations.hpp"
#include "aikido/planner/dart/SingleProblemPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Base planner class for ConfigurationToConfigurations planning problem.
class ConfigurationToConfigurationsPlanner
  : public dart::SingleProblemPlanner<
        ConfigurationToConfigurationsPlanner,
        ConfigurationToConfigurations>
{
public:
  // Expose the implementation of Planner::plan(const Problem&, Result*) in
  // SingleProblemPlanner. Note that plan() of the base class takes Problem
  // while the virtual function defined in this class takes SolvableProblem,
  // which is simply ConfigurationToConfigurations.
  using SingleProblemPlanner::plan;

  /// Constructor
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] metaSkeleton MetaSkeleton to use for planning.
  /// \param[in] configurationRanker Ranker to rank configurations.
  ConfigurationToConfigurationsPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      distance::ConstConfigurationRankerPtr configurationRanker = nullptr);

  /// Solves \c problem returning the result to \c result.
  /// NOTE: Because this is a DART planner, this method must both 1) lock the
  /// MetaSkeleton during planning and 2) reset the MetaSkeleton state after
  /// planning.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr)
      = 0;
  // Note: SolvableProblem is defined in SingleProblemPlanner.

protected:
  distance::ConstConfigurationRankerPtr mConfigurationRanker;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONSPLANNER_HPP_
