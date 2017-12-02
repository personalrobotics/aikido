#ifndef AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/optimization/Function.hpp"
#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace optimization {

// trajectory::SplinePtr planOptimization(
//    const std::shared_ptr<statespace::dart::MetaSkeletonStateSpace>&
//    stateSpace,
//    const statespace::StateSpace::State* startState,
//    const statespace::StateSpace::State* goalState,
//    planner::PlanningResult& planningResult);

// TODO(JS): templatize this class for the trajectory type. For now, this class
// returns trajectory::Spline()
class OptimizationBasedMotionPlanning
{
public:
  OptimizationBasedMotionPlanning(const TrajectoryVariable& variablesToClone);

  ~OptimizationBasedMotionPlanning() = default;

  trajectory::TrajectoryPtr plan();

  void setVariable(const Variable& variableToClone);

  void setStartState(const statespace::StateSpace::State* startState);

  const statespace::StateSpace::State* getStartState() const;

  void setGoalState(const statespace::StateSpace::State* goalState);

  const statespace::StateSpace::State* getGoalState() const;

  /// Sets an objective function that should be minimized while satisfying the
  /// trajectory optimization constraint.
  ///
  /// Pass in a nullptr to remove the objective and make it a constant-zero
  /// function.
  ///
  /// \param[in] objective TODO
  void setObjective(const FunctionPtr& objective);

  /// Returns the objective function for this trajectory optimization.
  FunctionPtr getObjective();

  /// Returns the objective function for this trajectory optimization.
  ConstFunctionPtr getObjective() const;

  void setInitialGuess(const Eigen::VectorXd& guess);

  void setInitialGuess(const Variable& guess);

  /// Returns the Problem that is being maintained by this trajectory
  /// optimization.
  const std::shared_ptr<dart::optimizer::Problem>& getProblem();

  /// Returns the Problem that is being maintained by this trajectory
  /// optimization.
  std::shared_ptr<const dart::optimizer::Problem> getProblem() const;

  /// Sets the Solver that should be used by this trajectory optimization, and
  /// set it up with the Problem that is configured by this trajectory
  /// optimization.
  void setSolver(const std::shared_ptr<dart::optimizer::Solver>& newSolver);

  /// Returns the Solver that is being used by this trajectory optimization.
  const std::shared_ptr<dart::optimizer::Solver>& getSolver();

  /// Returns the Solver that is being used by this trajectory optimization.
  std::shared_ptr<const dart::optimizer::Solver> getSolver() const;

protected:
  void resetProblem(bool clearSeeds = false);

  VariablePtr mVariable;

  /// The Problem that will be maintained by this trajectory optimization.
  std::shared_ptr<dart::optimizer::Problem> mProblem;

  /// The solver that this trajectory optimization module will use for iterative
  /// methods.
  std::shared_ptr<dart::optimizer::Solver> mSolver;

  /// Objective for the trajectory optimization.
  FunctionPtr mObjective;

  const statespace::StateSpace::State* mStartState;

  const statespace::StateSpace::State* mGoalState;

  std::shared_ptr<trajectory::Trajectory> mTrajectory;

private:
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_
