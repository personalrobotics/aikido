#ifndef AIKIDO_PLANNER_CONFIGURATIONTOTSRPLANNER_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOTSRPLANNER_HPP_

#include "aikido/planner/ConfigurationToTSR.hpp"
#include "aikido/planner/SingleProblemPlanner.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

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
  /// \param[in] stateSpace State space that this planner is associated with.
  explicit ConfigurationToTSRPlanner(statespace::ConstStateSpacePtr stateSpace);

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr)
      = 0;
  // Note: SolvableProblem is defined in SingleProblemPlanner.
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOTSRPLANNER_HPP_
