#include "aikido/planner/SnapPlanner.hpp"

#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace planner {

trajectory::InterpolatedPtr planSnap(
    const statespace::ConstStateSpacePtr& stateSpace,
    const aikido::statespace::StateSpace::State* startState,
    const aikido::statespace::StateSpace::State* goalState,
    const std::shared_ptr<aikido::statespace::Interpolator>& interpolator,
    const std::shared_ptr<aikido::constraint::Testable>& constraint,
    aikido::planner::PlanningResult& planningResult)
{
  auto problem = ConfigurationToConfiguration(
      stateSpace, *startState, *goalState, constraint);

  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, interpolator);

  return planner->plan(problem, &planningResult);
}

} // namespace planner
} // namespace aikido
