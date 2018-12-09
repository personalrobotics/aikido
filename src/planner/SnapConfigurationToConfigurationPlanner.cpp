#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"

#include "aikido/common/VanDerCorput.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {

//==============================================================================
SnapConfigurationToConfigurationPlanner::
    SnapConfigurationToConfigurationPlanner(
        statespace::ConstStateSpacePtr stateSpace,
        statespace::ConstInterpolatorPtr interpolator)
  : ConfigurationToConfigurationPlanner(std::move(stateSpace))
  , mInterpolator(std::move(interpolator))
{
  if (!mInterpolator)
  {
    mInterpolator
        = std::make_shared<statespace::GeodesicInterpolator>(mStateSpace);
  }
}

//==============================================================================
trajectory::TrajectoryPtr SnapConfigurationToConfigurationPlanner::plan(
    const SolvableProblem& problem, Result* result)
{
  // TODO(JS): Check equality between state space of this planner and given
  // problem.

  auto returnTraj
      = std::make_shared<trajectory::Interpolated>(mStateSpace, mInterpolator);
  auto testState = mStateSpace->createState();
  auto startState = problem.getStartState();
  auto goalState = problem.getGoalState();
  auto constraint = problem.getConstraint();

  aikido::common::VanDerCorput vdc{1, true, true, 0.02}; // TODO junk resolution
  for (const auto alpha : vdc)
  {
    mInterpolator->interpolate(startState, goalState, alpha, testState);
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

//==============================================================================
void SnapConfigurationToConfigurationPlanner::setInterpolator(
    statespace::ConstInterpolatorPtr interpolator)
{
  mInterpolator = std::move(interpolator);
}

//==============================================================================
statespace::ConstInterpolatorPtr
SnapConfigurationToConfigurationPlanner::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
PlannerPtr SnapConfigurationToConfigurationPlanner::clone(
    common::RNG* rng) const
{
  return std::make_shared<SnapConfigurationToConfigurationPlanner>
    (mStateSpace, mInterpolator);
}

//==============================================================================
bool SnapConfigurationToConfigurationPlanner::stopPlanning()
{
  //  Does not allow stop planning
  return false;
}

} // namespace planner
} // namespace aikido
