#include "aikido/planner/parabolic/ParabolicTimer.hpp"

#include <cassert>
#include <set>

#include "aikido/common/Spline.hpp"
#include "aikido/common/memory.hpp"
#include "aikido/trajectory/Interpolated.hpp"

#include "DynamicPath.h"
#include "ParabolicUtil.hpp"

using Eigen::Vector2d;

namespace aikido {
namespace planner {
namespace parabolic {

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeParabolicTiming(
    const aikido::trajectory::Interpolated& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();

  if (static_cast<std::size_t>(_maxVelocity.size()) != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (static_cast<std::size_t>(_maxAcceleration.size()) != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (std::size_t i = 0; i < dimension; ++i)
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

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = detail::convertToDynamicPath(
      _inputTrajectory, _maxVelocity, _maxAcceleration);

  auto outputTrajectory
      = detail::convertToSpline(*dynamicPath, startTime, stateSpace);
  return outputTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeParabolicTiming(
    const aikido::trajectory::Spline& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();

  if (static_cast<std::size_t>(_maxVelocity.size()) != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (static_cast<std::size_t>(_maxAcceleration.size()) != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (std::size_t i = 0; i < dimension; ++i)
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

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = detail::convertToDynamicPath(
      _inputTrajectory, _maxVelocity, _maxAcceleration, false);

  auto outputTrajectory
      = detail::convertToSpline(*dynamicPath, startTime, stateSpace);
  return outputTrajectory;
}

//==============================================================================
ParabolicTimer::ParabolicTimer(
    const Eigen::VectorXd& _velocityLimits,
    const Eigen::VectorXd& _accelerationLimits)
  : mVelocityLimits{_velocityLimits}, mAccelerationLimits{_accelerationLimits}
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ParabolicTimer::postprocess(
    const aikido::trajectory::Interpolated& _inputTraj,
    const aikido::common::RNG& /*_rng*/,
    const aikido::constraint::TestablePtr& /*_constraint*/)
{
  return computeParabolicTiming(
      _inputTraj, mVelocityLimits, mAccelerationLimits);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ParabolicTimer::postprocess(
    const aikido::trajectory::Spline& _inputTraj,
    const aikido::common::RNG& /*_rng*/,
    const aikido::constraint::TestablePtr& /*_constraint*/)
{
  return computeParabolicTiming(
      _inputTraj, mVelocityLimits, mAccelerationLimits);
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
