#ifndef AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_

#include "aikido/constraint.hpp"
#include "aikido/statespace.hpp"
#include "aikido/trajectory.hpp"
#include "aikido/planner/PlanningResult.hpp"

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

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_PLANNER_HPP_
