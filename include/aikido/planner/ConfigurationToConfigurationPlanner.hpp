#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_

#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/planner/PlanningResult.hpp"

namespace aikido {
namespace planner {

class ConfigurationToConfigurationPlanner
{
public:
  using Problem = ConfigurationToConfiguration;
  using Result = PlanningResult;

  virtual trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) = 0;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONPLANNER_HPP_
