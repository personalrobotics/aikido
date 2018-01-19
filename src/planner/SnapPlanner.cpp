#include "aikido/planner/SnapPlanner.hpp"

#include "aikido/common/VanDerCorput.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanToConfiguration.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/statespace/Interpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

trajectory::InterpolatedPtr planSnap(
    const std::shared_ptr<aikido::statespace::StateSpace>& stateSpace,
    const aikido::statespace::StateSpace::State* startState,
    const aikido::statespace::StateSpace::State* goalState,
    const std::shared_ptr<aikido::statespace::Interpolator>& interpolator,
    const std::shared_ptr<aikido::constraint::Testable>& constraint,
    aikido::planner::PlanningResult& planningResult)
{
  if (stateSpace != constraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
  aikido::common::VanDerCorput vdc{1, true, true, 0.02}; // TODO junk resolution
  auto returnTraj
      = std::make_shared<trajectory::Interpolated>(stateSpace, interpolator);
  auto testState = stateSpace->createState();

  for (const auto alpha : vdc)
  {
    interpolator->interpolate(startState, goalState, alpha, testState);
    if (!constraint->isSatisfied(testState))
    {
      planningResult.message = "Collision detected";
      return nullptr;
    }
  }

  returnTraj->addWaypoint(0, startState);
  returnTraj->addWaypoint(1, goalState);
  return returnTraj;
}

//==============================================================================
SnapPlanner::PlanningFunctionMap SnapPlanner::mPlanningFunctionMap;
bool SnapPlanner::mRegisteredPlanningFunctions = false;

//==============================================================================
SnapPlanner::SnapPlanner()
{
  if (!mRegisteredPlanningFunctions)
  {
    //    registerPlanningFunction<PlanToConfiguration>(&SnapPlanner::solve);
  }
}

//==============================================================================
trajectory::InterpolatedPtr SnapPlanner::planToConfiguration(
    const PlanToConfiguration* problem, PlanToConfiguration::Result* result)
{
  return nullptr;

  //  // TODO(JS): nullity check for problem

  //  aikido::common::VanDerCorput vdc{1, true, true, 0.02}; // TODO junk
  //  resolution

  //  auto stateSpace = problem->getStateSpace();
  //  auto interpolator = problem->getInterpolator();
  //  auto returnTraj
  //      = std::make_shared<trajectory::Interpolated>(stateSpace,
  //      interpolator);
  //  auto testState = stateSpace->createState();
  //  auto startState = problem->getStartState();
  //  auto goalState = problem->getGoalState();
  //  auto constraint = problem->getConstraint();

  //  for (const auto alpha : vdc)
  //  {
  //    interpolator->interpolate(startState, goalState, alpha, testState);
  //    if (!constraint->isSatisfied(testState))
  //    {
  //      if (result)
  //        result->setMessage("Collision detected");

  //      return nullptr;
  //    }
  //  }

  //  returnTraj->addWaypoint(0, startState);
  //  returnTraj->addWaypoint(1, goalState);

  //  return returnTraj;
}

//==============================================================================
trajectory::InterpolatedPtr SnapPlanner::planToConfiguration(
    const Problem* problem, Problem::Result* result)
{
  const auto* castedProblem = dynamic_cast<const PlanToConfiguration*>(problem);
  if (!castedProblem)
    throw std::invalid_argument("problem is not PlanToConfiguration type");

  if (result)
  {
    auto* castedResult = dynamic_cast<PlanToConfiguration::Result*>(result);
    if (!castedResult)
    {
      throw std::invalid_argument(
          "result is not PlanToConfiguration::Result type");
    }
  }
}

//==============================================================================
SnapPlanner::PlanningFunctionMap& SnapPlanner::getPlanningFunctionMap()
{
  return mPlanningFunctionMap;
}

} // namespace planner
} // namespace aikido
