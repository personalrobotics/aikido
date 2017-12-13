#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZER_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZER_HPP_

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
class Optimizer
{
public:
  struct OutCome;

  explicit Optimizer(const TrajectoryVariable& variablesToClone);

  virtual ~Optimizer() = default;

  std::shared_ptr<OutCome> createOutCome() const;

  VariableUniquePtr solve(OutCome* outcome = nullptr);

  void setVariable(const Variable& variableToClone);

  /// Sets an objective function that should be minimized while satisfying the
  /// trajectory optimization constraint.
  ///
  /// Pass in a nullptr to remove the objective and make it a constant-zero
  /// function.
  void setObjective(const FunctionPtr& objective);

  /// Returns the objective function for this trajectory optimization.
  FunctionPtr getObjective();

  /// Returns the objective function for this trajectory optimization.
  ConstFunctionPtr getObjective() const;

  void setInitialGuess(const Eigen::VectorXd& guess);

  void setInitialGuess(const Variable& guess);

  void setLowerBounds(const Eigen::VectorXd& lowerBounds);

  void setLowerBounds(const Variable& lowerBounds);

  void setUpperBounds(const Eigen::VectorXd& upperBounds);

  void setUpperBounds(const Variable& upperBounds);

  /// Returns the Problem that is being maintained by this trajectory
  /// optimization.
  std::shared_ptr<const dart::optimizer::Problem> getProblem() const;
  // We return as const problem because we don't want the problem to be changed
  // externally.

  /// Sets the Solver that should be used by this trajectory optimization, and
  /// set it up with the Problem that is configured by this trajectory
  /// optimization.
  void setSolver(const std::shared_ptr<dart::optimizer::Solver>& newSolver);

  /// Returns the Solver that is being used by this trajectory optimization.
  const std::shared_ptr<dart::optimizer::Solver>& getSolver();

  /// Returns the Solver that is being used by this trajectory optimization.
  std::shared_ptr<const dart::optimizer::Solver> getSolver() const;

  void setMaxAttempts(std::size_t maxAttempts);

  std::size_t getMaxAttempts() const;

  /// Add a seed for the Solver to use as a hint for the neighborhood of
  /// the solution.
  void addSeed(const Eigen::VectorXd& seed);

  /// Get a mutable reference of the seed for the specified index. If an
  /// out-of-bounds index is provided a warning will print, and a reference to
  /// the initial guess will be returned instead.
  Eigen::VectorXd& getSeed(std::size_t index);

  /// An immutable version of getSeed(std::size_t)
  const Eigen::VectorXd& getSeed(std::size_t index) const;

  /// Get a mutable reference to the full vector of seeds that this
  /// Problem currently contains
  std::vector<Eigen::VectorXd>& getSeeds();

  /// An immutable version of getSeeds()
  const std::vector<Eigen::VectorXd>& getSeeds() const;

  /// Clear the seeds that this Problem currently contains
  void clearAllSeeds();

protected:
  void randomizeConfiguration(Eigen::VectorXd& x);

  VariablePtr mVariable;

  /// The Problem that will be maintained by this trajectory optimization.
  std::shared_ptr<dart::optimizer::Problem> mProblem;

  /// The solver that this trajectory optimization module will use for iterative
  /// methods.
  std::shared_ptr<dart::optimizer::Solver> mSolver;

  /// Objective for the trajectory optimization.
  FunctionPtr mObjective;

  std::shared_ptr<trajectory::Trajectory> mTrajectory;

  std::size_t mMaxAttempts;
};

struct Optimizer::OutCome
{
  Eigen::VectorXd mInitialGuess;

  double mInitialFunctionEvaluation;

  bool mMinimized;

  Eigen::VectorXd mSolution;
  // TODO(JS): Consider having multiple solutions

  double mMinimalFunctionEvaluation;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZER_HPP_
