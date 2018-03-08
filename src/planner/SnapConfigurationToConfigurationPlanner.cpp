#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"

#include "aikido/common/VanDerCorput.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {

//==============================================================================
trajectory::TrajectoryPtr SnapConfigurationToConfigurationPlanner::plan(
    const Problem& problem, Result* result)
{
  auto stateSpace = problem.getStateSpace();
  auto interpolator = problem.getInterpolator();
  auto returnTraj
      = std::make_shared<trajectory::Interpolated>(stateSpace, interpolator);
  auto testState = stateSpace->createState();
  auto startState = problem.getStartState();
  auto goalState = problem.getGoalState();
  auto constraint = problem.getConstraint();

  aikido::common::VanDerCorput vdc{1, true, true, 0.02}; // TODO junk resolution
  for (const auto alpha : vdc)
  {
    interpolator->interpolate(startState, goalState, alpha, testState);
    if (!constraint->isSatisfied(testState))
    {
      if (result)
        result->setMessage("Collision detected");

      return nullptr;
    }
  }

  returnTraj->addWaypoint(0, startState);
  returnTraj->addWaypoint(1, goalState);

  return returnTraj;
}

} // namespace planner
} // namespace aikido
