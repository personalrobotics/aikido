#ifndef AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace optimization {

trajectory::InterpolatedPtr planOptimization(
    const std::shared_ptr<statespace::StateSpace>& stateSpace,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    const std::shared_ptr<statespace::Interpolator>& interpolator,
    const std::shared_ptr<constraint::Testable>& constraint,
    planner::PlanningResult& planningResult);

class OptimizationBasedMotionPlanning
{
public:
  OptimizationBasedMotionPlanning() = default;
  ~OptimizationBasedMotionPlanning() = default;

  virtual bool plan();

protected:
  /// The Problem that will be maintained by this IK module
  std::shared_ptr<dart::optimizer::Problem> mProblem;

  /// The solver that this IK module will use for iterative methods
  std::shared_ptr<dart::optimizer::Solver> mSolver;

private:
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_
