#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRPLANNER_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRPLANNER_HPP_

#include "aikido/planner/SingleProblemPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToTSR.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Base planner class for ConfigurationToTSR planning problem.
class ConfigurationToTSRPlanner
    : public SingleProblemPlanner<ConfigurationToTSRPlanner, ConfigurationToTSR>
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
  ConfigurationToTSRPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton);

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr)
      = 0;
  // Note: SolvableProblem is defined in SingleProblemPlanner.

  /// Return this planner's MetaSkeletonStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr getMetaSkeletonStateSpace();

protected:
  /// Stores stateSpace pointer as a ConstMetaSkeletonStateSpacePtr. Prevents
  /// use of an expensive dynamic cast.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// MetaSkeleton to use for planning.
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOTSRPLANNER_HPP_
