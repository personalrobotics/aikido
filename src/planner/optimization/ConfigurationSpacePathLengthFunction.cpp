#include "aikido/planner/optimization/ConfigurationSpacePathLengthFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
ConfigurationSpacePathLengthFunction::ConfigurationSpacePathLengthFunction(
    const distance::DistanceMetricPtr& distanceMetric)
  : mDistanceMetric(distanceMetric)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<Function> ConfigurationSpacePathLengthFunction::clone() const
{
  return std::make_shared<ConfigurationSpacePathLengthFunction>(mDistanceMetric);
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
