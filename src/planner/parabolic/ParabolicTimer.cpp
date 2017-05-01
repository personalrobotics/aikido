#include <cassert>
#include <set>
#include <dart/common/StlHelpers.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/util/Spline.hpp>
#include "DynamicPath.h"
#include "ParabolicUtil.hpp"

using Eigen::Vector2d;
using dart::common::make_unique;

using CubicSplineProblem
    = aikido::util::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

namespace aikido {
namespace planner {
namespace parabolic {


std::unique_ptr<aikido::trajectory::Spline> computeParabolicTiming(
      aikido::trajectory::Spline* _inputTrajectory,
      const Eigen::VectorXd& _maxVelocity,
      const Eigen::VectorXd& _maxAcceleration)
{
  const auto stateSpace = _inputTrajectory->getStateSpace();
  const auto dimension = stateSpace->getDimension();

  if (static_cast<size_t>(_maxVelocity.size()) != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (static_cast<size_t>(_maxAcceleration.size()) != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (size_t i = 0; i < dimension; ++i)
  {
    if (_maxVelocity[i] <= 0.)
      throw std::invalid_argument("Velocity limits must be positive.");
    if (!std::isfinite(_maxVelocity[i]))
      throw std::invalid_argument("Velocity limits must be finite.");

    if (_maxAcceleration[i] <= 0.)
      throw std::invalid_argument("Acceleration limits must be positive.");
    if (!std::isfinite(_maxAcceleration[i]))
      throw std::invalid_argument("Acceleration limits must be finite.");
  }

  ParabolicRamp::DynamicPath dynamicPath;
  // Apply the adjoint limits
  dynamicPath.Init(toVector(_maxVelocity), toVector(_maxAcceleration));

  double startTime = 0.0;
  convertToDynamicPath(_inputTrajectory, dynamicPath, startTime);

  aikido::trajectory::Spline* timedTrajectory = convertToSpline(dynamicPath,
                                                                startTime,
                                                                stateSpace);
  std::unique_ptr<aikido::trajectory::Spline> outputTrajectory(timedTrajectory);
  return outputTrajectory;
}

std::unique_ptr<trajectory::Spline> computeParabolicTiming(
    const trajectory::Interpolated& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration)
{
  aikido::trajectory::Spline* spline = convertToSpline(_inputTrajectory);
  std::unique_ptr<trajectory::Spline> timedSpline =
          computeParabolicTiming(spline, _maxVelocity, _maxAcceleration);
  return timedSpline;
}


} // namespace parabolic
} // namespace planner
} // namespace aikido
