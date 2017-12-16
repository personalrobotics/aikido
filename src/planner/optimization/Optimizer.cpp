#include "aikido/planner/optimization/Optimizer.hpp"

#include <dart/dart.hpp>
#include <dart/optimizer/nlopt/nlopt.hpp>
#include <dart/optimizer/optimizer.hpp>

#include "aikido/common/VanDerCorput.hpp"
#include "aikido/planner/optimization/TrajectoryVariable.hpp"
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
//  Optimizer planner(stateSpace);

//  planner.setStartState(startState);
//  planner.setGoalState(goalState);

//  auto returnTraj = planner.plan();

//  return returnTraj;
//}

//==============================================================================
Optimizer::Optimizer(const Variable& variablesToClone)
  : mVariable(nullptr), mMaxAttempts(16u)
{
  mProblem = std::make_shared<dart::optimizer::Problem>(0);
  mSolver = std::make_shared<dart::optimizer::NloptSolver>(
      mProblem, nlopt::LN_COBYLA);
  // TODO(JS): Choose algorithm according to the problem.

  setVariable(variablesToClone);
}

//==============================================================================
std::shared_ptr<Optimizer::OutCome> Optimizer::createOutCome() const
{
  return std::make_shared<OutCome>();
}

//==============================================================================
UniqueVariablePtr Optimizer::solve(OutCome* outcome)
{
  if (nullptr == mSolver)
  {
    dtwarn << "[Optimizer::plan] The Solver for an "
           << "Optimizer module is a nullptr. You must "
           << "reset the Solver before you can use it.\n";
    return nullptr;
  }

  mProblem->setObjective(mObjective);

  auto f0 = mObjective->eval(mProblem->getInitialGuess());

  Eigen::VectorXd x0 = mProblem->getInitialGuess();
  Eigen::VectorXd savedX0 = x0;

  std::size_t attemptCount = 0u;
  bool minimized = false;
  do
  {
    mSolver->setNumMaxIterations(1024); // TODO(JS): Make this as a parameter
    minimized = mSolver->solve();

    if (!minimized)
    {
      ++attemptCount;

      if (mMaxAttempts > 0 && attemptCount >= mMaxAttempts)
        break;

      if (attemptCount - 1 < mProblem->getSeeds().size())
        x0 = mProblem->getSeed(attemptCount - 1);
      else
        randomizeConfiguration(x0);
    }

  } while (!minimized);

  const Eigen::VectorXd& solution = mProblem->getOptimalSolution();

  // TODO(JS): do have some backup plans when it fails to solve the problem
  // by nlopt.

  auto f1 = mObjective->eval(solution);

  // TODO(JS): return success
  if (outcome)
  {
    outcome->mInitialGuess = mProblem->getInitialGuess();
    outcome->mInitialFunctionEvaluation = f0;
    outcome->mMinimized = minimized;
    outcome->mSolution = solution;
    outcome->mMinimalFunctionEvaluation = f1;
  }

  mProblem->setInitialGuess(savedX0);

  if (minimized)
  {
    auto solution = mVariable->clone();
    solution->setValue(mProblem->getOptimalSolution());
    return solution;
  }
  else
  {
    return nullptr;
  }
}

//==============================================================================
void Optimizer::setVariable(const Variable& variableToClone)
{
  mVariable = variableToClone.clone();

  if (mObjective)
    mObjective->setVariable(mVariable);

  assert(mProblem);
  mProblem->setDimension(mVariable->getDimension());
  mProblem->setInitialGuess(mVariable->getValue());

  for (auto i = 0u; i < mProblem->getNumEqConstraints(); ++i)
  {
    auto aikidoFunction
        = std::static_pointer_cast<Function>(mProblem->getEqConstraint(i));
    aikidoFunction->setVariable(mVariable);
  }
}

//==============================================================================
void Optimizer::setObjective(const FunctionPtr& objective)
{
  mObjective = objective;
  mObjective->setVariable(mVariable);
}

//==============================================================================
FunctionPtr Optimizer::getObjective()
{
  return mObjective;
}

//==============================================================================
ConstFunctionPtr Optimizer::getObjective() const
{
  return mObjective;
}

//==============================================================================
void Optimizer::setInitialGuess(const Eigen::VectorXd& guess)
{
  // TODO(JS): Check if any of the values is whether invalid values such as
  // nan, (not sure but inf or -inf).
  mProblem->setInitialGuess(guess);
}

//==============================================================================
void Optimizer::setInitialGuess(const Variable& guess)
{
  setInitialGuess(guess.getValue());
}

//==============================================================================
void Optimizer::setLowerBounds(const Eigen::VectorXd& lowerBounds)
{
  mProblem->setLowerBounds(lowerBounds);
}

//==============================================================================
void Optimizer::setLowerBounds(const Variable& lowerBounds)
{
  setLowerBounds(lowerBounds.getValue());
}

//==============================================================================
void Optimizer::setUpperBounds(const Eigen::VectorXd& upperBounds)
{
  mProblem->setUpperBounds(upperBounds);
}

//==============================================================================
void Optimizer::setUpperBounds(const Variable& upperBounds)
{
  setUpperBounds(upperBounds.getValue());
}

//==============================================================================
std::shared_ptr<const dart::optimizer::Problem> Optimizer::getProblem() const
{
  return mProblem;
}

//==============================================================================
void Optimizer::setSolver(
    const std::shared_ptr<dart::optimizer::Solver>& newSolver)
{
  mSolver = newSolver;

  if (mSolver)
    mSolver->setProblem(mProblem);
}

//==============================================================================
const std::shared_ptr<dart::optimizer::Solver>& Optimizer::getSolver()
{
  return mSolver;
}

//==============================================================================
std::shared_ptr<const dart::optimizer::Solver> Optimizer::getSolver() const
{
  return mSolver;
}

//==============================================================================
void Optimizer::setMaxAttempts(std::size_t maxAttempts)
{
  mMaxAttempts = maxAttempts;
}

//==============================================================================
std::size_t Optimizer::getMaxAttempts() const
{
  return mMaxAttempts;
}

//==============================================================================
void Optimizer::addSeed(const Eigen::VectorXd& seed)
{
  mProblem->addSeed(seed);
}

//==============================================================================
Eigen::VectorXd& Optimizer::getSeed(std::size_t index)
{
  return mProblem->getSeed(index);
}

//==============================================================================
const Eigen::VectorXd& Optimizer::getSeed(std::size_t index) const
{
  return mProblem->getSeed(index);
}

//==============================================================================
std::vector<Eigen::VectorXd>& Optimizer::getSeeds()
{
  return mProblem->getSeeds();
}

//==============================================================================
const std::vector<Eigen::VectorXd>& Optimizer::getSeeds() const
{
  return mProblem->getSeeds();
}

//==============================================================================
void Optimizer::clearAllSeeds()
{
  mProblem->clearAllSeeds();
}

//==============================================================================
void Optimizer::randomizeConfiguration(Eigen::VectorXd& x)
{
  const auto& lb = mProblem->getLowerBounds();
  const auto& ub = mProblem->getUpperBounds();

  // TODO(JS): Ask to each variables to be able to generate random value(?)
  mVariable->generateRandomValueTo(x, lb, ub);
}

} // namespace optimization
} // namespace planner
} // namespace aikido
