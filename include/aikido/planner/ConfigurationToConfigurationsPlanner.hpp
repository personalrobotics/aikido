#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONSPLANNER_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONSPLANNER_HPP_

#include "aikido/planner/ConfigurationToConfigurations.hpp"
#include "aikido/planner/SingleProblemPlanner.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// Base planner class for ConfigurationToConfigurations planning problem.
class ConfigurationToConfigurationsPlanner
  : public SingleProblemPlanner<
        ConfigurationToConfigurationsPlanner,
        ConfigurationToConfigurations>
{
public:
  // Expose the implementation of Planner::plan(const Problem&, Result*) in
  // SingleProblemPlanner. Note that plan() of the base class takes Problem
  // while the virtual function defined in this class takes SolvableProblem,
  // which is simply ConfigurationToConfigurations.
  using SingleProblemPlanner::plan;

  /// \copydoc Planner::Planner
  explicit ConfigurationToConfigurationsPlanner(
      statespace::ConstStateSpacePtr stateSpace, common::RNG* rng = nullptr);

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

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONSPLANNER_HPP_
