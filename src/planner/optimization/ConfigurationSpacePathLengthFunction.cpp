#include "aikido/planner/optimization/ConfigurationSpacePathLengthFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
ConfigurationSpacePathLengthFunction::ConfigurationSpacePathLengthFunction(
    TrajectoryVariablePtr variable,
    const distance::DistanceMetricPtr& distanceMetric)
  : TrajectoryFunction(std::move(variable)), mDistanceMetric(distanceMetric)
{
  // Do nothing
}

//==============================================================================
UniqueFunctionPtr ConfigurationSpacePathLengthFunction::clone() const
{
  return dart::common::make_unique<ConfigurationSpacePathLengthFunction>(
      getTrajectoryVariable(), mDistanceMetric);
}

//==============================================================================
double ConfigurationSpacePathLengthFunction::eval(const Eigen::VectorXd& x)
{
  mVariable->setValue(x);

  const auto& trajectory = getTrajectory();
  const auto arcLength = trajectory.computeArcLength(mDistanceMetric.get());

  return arcLength;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
