#include "aikido/planner/optimization/TrajectoryOptimizer.hpp"

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
TrajectoryOptimizer::TrajectoryOptimizer(
    const TrajectoryVariable& variablesToClone)
  : Optimizer(variablesToClone)
{
  if (!variablesToClone.isTrajectoryVariable())
  {
    throw std::invalid_argument(
        "Variable should be a type of TrajectoryVariable");
  }
}

//==============================================================================
trajectory::TrajectoryPtr TrajectoryOptimizer::plan(OutCome* outcome)
{
  auto variable = solve(outcome);
  auto trajectoryVariable = static_cast<TrajectoryVariable*>(variable.get());

  return trajectoryVariable->getTrajectory().clone();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
