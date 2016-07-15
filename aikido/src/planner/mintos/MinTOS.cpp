#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <mintos/MinTOS.h>

namespace aikido {
namespace planner {
namespace mintos {

std::unique_ptr<trajectory::Spline> interpolateAndTimeOptimizeTrajectory(
  const trajectory::Interpolated& _inputTrajectory,
  const Eigen::VectorXd& _minVelocity,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _minAcceleration,
  const Eigen::VectorXd& _maxAcceleration,
  double _constriantTolerance)
{
  std::vector<Mintos::Config> milestones; // TODO: Populate this.
  Math::VectorFieldFunction* constraint; // TODO: Define this.
  Mintos::Vector minVelocity, maxVelocity;
  Mintos::Vector minAcceleration, maxAcceleration;
  Mintos::TimeScaledBezierCurve outputCurve;

  auto const success = Mintos::InterpolateAndTimeOptimize(
    milestones,
    nullptr, // TODO: Replace this with a GeodesicManifold from the StateSpace.
    constraint,
    _constriantTolerance,
    minVelocity, maxVelocity,
    minAcceleration, maxAcceleration,
    outputCurve);
  if (!success)
    throw std::runtime_error("Failed to optimize input path.");

  return nullptr;
}

} // namespace mintos
} // namespace planner
} // namespace aikido
