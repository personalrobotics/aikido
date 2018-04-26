#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_

#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/PlannerForSingleProblem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

class ConfigurationToConfigurationPlanner
    : public PlannerForSingleProblem<ConfigurationToConfigurationPlanner,
                                     ConfigurationToConfiguration>
{
public:
  using TheProblem = ConfigurationToConfiguration;
  using TheResult = Result;

  /// Constructor
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  ConfigurationToConfigurationPlanner(
      const statespace::ConstStateSpacePtr& stateSpace);

  // TODO(JS): Docstring
  virtual trajectory::TrajectoryPtr plan(
      const TheProblem& problem, TheResult* result = nullptr)
      = 0;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
