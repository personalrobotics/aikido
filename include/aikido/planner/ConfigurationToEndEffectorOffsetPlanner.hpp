#ifndef AIKIDO_PLANNER_CONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_

#include "aikido/planner/ConfigurationToEndEffectorOffset.hpp"
#include "aikido/planner/SingleProblemPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// Base planner class for ConfigurationToEndEffectorOffset planning problem.
class ConfigurationToEndEffectorOffsetPlanner
    : public SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                                  ConfigurationToEndEffectorOffset>
{
public:
  // Expose the implementation of Planner::plan(const Problem&, Result*) in
  // SingleProblemPlanner. Note that plan() of the base class takes Problem
  // while the virtual function defined in this class takes SolverbleProblem,
  // which is simply ConfigurationToEndEffectorOffset.
  using SingleProblemPlanner::plan;

  /// Constructor
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  explicit ConfigurationToEndEffectorOffsetPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace);

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr)
      = 0;
  // Note: SolvableProblem is defined in SingleProblemPlanner.

  /// Return ConstMetaSkeletonStateSpacePtr by performing a static cast on
  /// mStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr getMetaSkeletonStateSpace();

protected:
  /// Stores stateSpace pointer as aConstMetaSkeletonStateSpacePtr. Prevents
  /// use of an expensive dynamic cast.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_
