#include "aikido/planner/optimization/Planner.hpp"

#include <dart/dart.hpp>
#include <dart/optimizer/nlopt/nlopt.hpp>
#include <dart/optimizer/optimizer.hpp>

#include "aikido/common/VanDerCorput.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
trajectory::InterpolatedPtr planOptimization(
    const std::shared_ptr<statespace::StateSpace>& stateSpace,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    const std::shared_ptr<statespace::Interpolator>& interpolator,
    const std::shared_ptr<constraint::Testable>& constraint,
    planner::PlanningResult& planningResult)
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
bool OptimizationBasedMotionPlanning::plan()
{
  if (nullptr == mSolver)
  {
    dtwarn << "[OptimizationBasedMotionPlanning::plan] The Solver for an "
           << "OptimizationBasedMotionPlanning module is a nullptr. You must "
           << "reset the module's Solver before you can use it.\n";
    return false;
  }

  if (nullptr == mProblem)
  {
    dtwarn << "[OptimizationBasedMotionPlanning::plan] The Problem for an "
           << "OptimizationBasedMotionPlanning module is a nullptr. You must "
           << "reset the module's Solver before you can use it.\n";
    return false;
  }

  //  mProblem->setDimension(mDofs.size());

  //  mProblem->setInitialGuess(getPositions());

  //  const dart::dynamics::SkeletonPtr& skel = getNode()->getSkeleton();

  //  Eigen::VectorXd bounds(mDofs.size());
  //  for(std::size_t i=0; i < mDofs.size(); ++i)
  //    bounds[i] = skel->getDof(mDofs[i])->getPositionLowerLimit();
  //  mProblem->setLowerBounds(bounds);

  //  for(std::size_t i=0; i < mDofs.size(); ++i)
  //    bounds[i] = skel->getDof(mDofs[i])->getPositionUpperLimit();
  //  mProblem->setUpperBounds(bounds);

  //  // Many GradientMethod implementations use Joint::integratePositions, so
  //  we
  //  // need to clear out any velocities that might be in the Skeleton and then
  //  // reset those velocities later. This has been opened as issue #699.
  //  Eigen::VectorXd originalVelocities = skel->getVelocities();
  //  for(std::size_t i=0; i < skel->getNumDofs(); ++i)
  //    skel->getDof(i)->setVelocity(0.0);

  //  Eigen::VectorXd originalPositions = getPositions();
  bool wasSolved = mSolver->solve();
  //  setPositions(originalPositions);
  //  skel->setVelocities(originalVelocities);

  return wasSolved;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
