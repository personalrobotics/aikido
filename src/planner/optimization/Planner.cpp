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
// trajectory::SplinePtr planOptimization(
//    const std::shared_ptr<statespace::dart::MetaSkeletonStateSpace>&
//    stateSpace,
//    const statespace::StateSpace::State* startState,
//    const statespace::StateSpace::State* goalState,
//    planner::PlanningResult& /*planningResult*/)
//{
//  OptimizationBasedMotionPlanning planner(stateSpace);

//  planner.setStartState(startState);
//  planner.setGoalState(goalState);

//  auto returnTraj = planner.plan();

//  return returnTraj;
//}

//==============================================================================
OptimizationBasedMotionPlanning::OptimizationBasedMotionPlanning(
    const TrajectoryVariable& variablesToClone)
  : mVariable(nullptr)
{
  mProblem = std::make_shared<dart::optimizer::Problem>(0);
  mSolver = std::make_shared<dart::optimizer::NloptSolver>(
      mProblem, nlopt::LN_COBYLA);
  // TODO(JS): Choose algorithm according to the problem.

  setVariable(variablesToClone);

  resetProblem();
}

//==============================================================================
trajectory::TrajectoryPtr OptimizationBasedMotionPlanning::plan()
{
  if (nullptr == mSolver)
  {
    dtwarn << "[OptimizationBasedMotionPlanning::plan] The Solver for an "
           << "OptimizationBasedMotionPlanning module is a nullptr. You must "
           << "reset the Solver before you can use it.\n";
    return nullptr;
  }

  if (nullptr == mProblem)
  {
    dtwarn << "[OptimizationBasedMotionPlanning::plan] The Problem for an "
           << "OptimizationBasedMotionPlanning module is a nullptr. You must "
           << "reset the Solver before using it.\n";
    return nullptr;
  }

  mProblem->setObjective(mObjective);

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
  const Eigen::VectorXd& solution = mProblem->getOptimalSolution();

  return nullptr;
}

//==============================================================================
void OptimizationBasedMotionPlanning::setVariable(
    const Variable& variableToClone)
{
  mVariable = variableToClone.clone();

  if (mObjective)
  {
    mObjective->setVariable(mVariable);
  }

  if (mProblem)
  {
    mProblem->setInitialGuess(mVariable->getValue());
  }
}

//==============================================================================
void OptimizationBasedMotionPlanning::setStartState(
    const statespace::StateSpace::State* startState)
{
  mStartState = startState;
}

//==============================================================================
const statespace::StateSpace::State*
OptimizationBasedMotionPlanning::getStartState() const
{
  return mStartState;
}

//==============================================================================
void OptimizationBasedMotionPlanning::setGoalState(
    const statespace::StateSpace::State* goalState)
{
  mGoalState = goalState;
}

//==============================================================================
const statespace::StateSpace::State*
OptimizationBasedMotionPlanning::getGoalState() const
{
  return mGoalState;
}

//==============================================================================
void OptimizationBasedMotionPlanning::setObjective(const FunctionPtr& objective)
{
  mObjective = objective;
  mObjective->setVariable(mVariable);
}

//==============================================================================
FunctionPtr OptimizationBasedMotionPlanning::getObjective()
{
  return mObjective;
}

//==============================================================================
ConstFunctionPtr OptimizationBasedMotionPlanning::getObjective() const
{
  return mObjective;
}

//==============================================================================
void OptimizationBasedMotionPlanning::setInitialGuess(const Eigen::VectorXd& guess)
{
  mProblem->setInitialGuess(guess);
}

//==============================================================================
void OptimizationBasedMotionPlanning::setInitialGuess(const Variable& guess)
{
  setInitialGuess(guess.getValue());
}

//==============================================================================
const std::shared_ptr<dart::optimizer::Problem>&
OptimizationBasedMotionPlanning::getProblem()
{
  return mProblem;
}

//==============================================================================
std::shared_ptr<const dart::optimizer::Problem>
OptimizationBasedMotionPlanning::getProblem() const
{
  return mProblem;
}

//==============================================================================
void OptimizationBasedMotionPlanning::setSolver(
    const std::shared_ptr<dart::optimizer::Solver>& newSolver)
{
  mSolver = newSolver;
}

//==============================================================================
const std::shared_ptr<dart::optimizer::Solver>&
OptimizationBasedMotionPlanning::getSolver()
{
  return mSolver;
}

//==============================================================================
std::shared_ptr<const dart::optimizer::Solver>
OptimizationBasedMotionPlanning::getSolver() const
{
  return mSolver;
}

//==============================================================================
void OptimizationBasedMotionPlanning::resetProblem(bool clearSeeds)
{
  mProblem->removeAllEqConstraints();
  mProblem->removeAllIneqConstraints();

  if (clearSeeds)
    mProblem->clearAllSeeds();

  mProblem->setObjective(mObjective);
  //  mProblem->setEq

  mProblem->setDimension(mVariable->getDimension());
}

} // namespace optimization
} // namespace planner
} // namespace aikido
