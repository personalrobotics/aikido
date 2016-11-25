#include <aikido/constraint/Testable.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/statespace/Interpolator.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/util/VanDerCorput.hpp>

namespace aikido {
namespace planner {

trajectory::InterpolatedPtr planSnap(
  const std::shared_ptr<aikido::statespace::StateSpace>& stateSpace,
  const aikido::statespace::StateSpace::State *startState,
  const aikido::statespace::StateSpace::State *goalState,
  const std::shared_ptr<aikido::statespace::Interpolator>& interpolator,
  const std::shared_ptr<aikido::constraint::Testable>& constraint,
  aikido::planner::PlanningResult& planningResult)
{
  if (stateSpace != constraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
  aikido::util::VanDerCorput vdc{1, true, 0.02};  // TODO junk resolution
  auto returnTraj = std::make_shared<trajectory::Interpolated>(
      stateSpace, interpolator);
  auto testState = stateSpace->createState();

  for (const auto alpha : vdc) {
    interpolator->interpolate(startState, goalState, alpha, testState);
    if (!constraint->isSatisfied(testState)) {
      planningResult.message = "Collision detected";
      return returnTraj;
    }
  }

  returnTraj->addWaypoint(0, startState);
  returnTraj->addWaypoint(1, goalState);
  return returnTraj;
}

} // namespace planner
} // namespace aikido
