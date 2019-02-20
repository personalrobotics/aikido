#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRPLANNER_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRPLANNER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"
#include "aikido/planner/dart/ConfigurationToTSR.hpp"
#include "aikido/planner/dart/SingleProblemPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Base planner class for ConfigurationToTSR planning problem.
class ConfigurationToTSRPlanner
    : public dart::SingleProblemPlanner<ConfigurationToTSRPlanner,
                                        ConfigurationToTSR>
{
public:
  // Expose the implementation of Planner::plan(const Problem&, Result*) in
  // SingleProblemPlanner. Note that plan() of the base class takes Problem
  // while the virtual function defined in this class takes SolvableProblem,
  // which is simply ConfigurationToTSR.
  using SingleProblemPlanner::plan;

  /// Constructor
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] metaSkeleton MetaSkeleton to use for planning.
  /// \param[in] configurationRanker Ranker to rank configurations.
  ConfigurationToTSRPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      distance::ConstConfigurationRankerPtr configurationRanker = nullptr);

  /// Solves \c problem returning the result to \c result.
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

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRPLANNER_HPP_
