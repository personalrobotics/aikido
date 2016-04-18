#include <aikido/planner/parabolic/ParabolicTimer.hpp>

namespace aikido {
namespace planner {
namespace parabolic {

std::unique_ptr<path::SplineTrajectory2> computeParabolicTiming(
  const path::PiecewiseLinearTrajectory& _inputTrajectory,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _maxAcceleration)
{
  // TODO: Implement this.
  return nullptr;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
