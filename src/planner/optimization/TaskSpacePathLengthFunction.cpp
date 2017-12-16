#include "aikido/planner/optimization/TaskSpacePathLengthFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
TaskSpacePathLengthFunction::TaskSpacePathLengthFunction(
    TrajectoryVariablePtr variable)
  : TrajectoryFunction(std::move(variable))
{
  // Do nothing
}

//==============================================================================
UniqueFunctionPtr TaskSpacePathLengthFunction::clone() const
{
  return dart::common::make_unique<TaskSpacePathLengthFunction>(mVariable);
}

//==============================================================================
double TaskSpacePathLengthFunction::eval(const Eigen::VectorXd& x)
{
  mVariable->setValue(x);

  const auto& trajectory = getTrajectory();
  const auto arcLength = trajectory.computeArcLength(mDistanceMetric.get());

  return arcLength;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
