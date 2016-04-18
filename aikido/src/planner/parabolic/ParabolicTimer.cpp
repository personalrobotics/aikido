#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include "DynamicPath.h"

namespace aikido {
namespace planner {
namespace parabolic {

using statespace::GeodesicInterpolator;

static ParabolicRamp::Vector toVector(const Eigen::VectorXd& _x)
{
  ParabolicRamp::Vector output(_x.size());
  for (size_t i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

std::unique_ptr<path::SplineTrajectory2> computeParabolicTiming(
  const path::PiecewiseLinearTrajectory& _inputTrajectory,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _maxAcceleration)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();

  const auto interpolator = _inputTrajectory.getInterpolator();
  if (dynamic_cast<const GeodesicInterpolator*>(interpolator.get()) == nullptr)
    throw std::invalid_argument(
      "computeParabolicTiming only supports geodesic interpolation.");

  if (_maxVelocity.size() != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (_maxAcceleration.size() != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (size_t i = 0; i < dimension; ++i)
  {
    if (_maxVelocity[i] <= 0.)
      throw std::invalid_argument("Velocity limits must be positive.");

    if (_maxAcceleration[i] <= 0.)
      throw std::invalid_argument("Acceleration limits must be positive.");
  }

  std::vector<ParabolicRamp::Vector> milestones;
  // TODO: Populate milestones

  ParabolicRamp::DynamicPath dynamicPath;
  dynamicPath.Init(toVector(_maxVelocity), toVector(_maxAcceleration));
  dynamicPath.SetMilestones(milestones);

  // TODO: Construct a SplineTrajectory2 from dynamicPath.ramps

  throw std::runtime_error("Not implemented.");
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
