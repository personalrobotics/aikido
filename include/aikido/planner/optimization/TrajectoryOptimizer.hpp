#ifndef AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYOPTIMIZER_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYOPTIMIZER_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/optimization/Function.hpp"
#include "aikido/planner/optimization/Optimizer.hpp"
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

class TrajectoryOptimizer : public Optimizer
{
public:
  explicit TrajectoryOptimizer(const TrajectoryVariable& variablesToClone);

  trajectory::TrajectoryPtr plan(OutCome* outcome = nullptr);
  // TODO(JS): Create TrajectoryOptimizerOutCome
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYOPTIMIZER_HPP_
