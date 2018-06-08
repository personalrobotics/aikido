#include "aikido/planner/kinodynamic/KinodynamicTimer.hpp"
#include <dart/dart.hpp>
#include <aikido/common/Spline.hpp>

using Eigen::Vector2d;
using dart::common::make_unique;

using LinearSplineProblem
    = aikido::common::SplineProblem<double, int, 2, Eigen::Dynamic, 2>;

namespace aikido {
namespace planner {
namespace kinodynamic {

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Interpolated& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration)
{
  const auto stateSpace = inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();

  if (static_cast<std::size_t>(maxVelocity.size()) != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (static_cast<std::size_t>(maxAcceleration.size()) != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (std::size_t i = 0; i < dimension; ++i)
  {
    if (maxVelocity[i] <= 0.)
      throw std::invalid_argument("Velocity limits must be positive.");
    if (!std::isfinite(maxVelocity[i]))
      throw std::invalid_argument("Velocity limits must be finite.");

    if (maxAcceleration[i] <= 0.)
      throw std::invalid_argument("Acceleration limits must be positive.");
    if (!std::isfinite(maxAcceleration[i]))
      throw std::invalid_argument("Acceleration limits must be finite.");
  }

  double startTime = inputTrajectory.getStartTime();
  /*
  auto dynamicPath = detail::convertToDynamicPath(
      inputTrajectory, maxVelocity, maxAcceleration);

  auto outputTrajectory
      = detail::convertToSpline(*dynamicPath, startTime, stateSpace);
  return outputTrajectory;
  */
  return nullptr;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Spline& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration)
{
  const auto stateSpace = inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();

  if (static_cast<std::size_t>(maxVelocity.size()) != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (static_cast<std::size_t>(maxAcceleration.size()) != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (std::size_t i = 0; i < dimension; ++i)
  {
    if (maxVelocity[i] <= 0.)
      throw std::invalid_argument("Velocity limits must be positive.");
    if (!std::isfinite(maxVelocity[i]))
      throw std::invalid_argument("Velocity limits must be finite.");

    if (maxAcceleration[i] <= 0.)
      throw std::invalid_argument("Acceleration limits must be positive.");
    if (!std::isfinite(maxAcceleration[i]))
      throw std::invalid_argument("Acceleration limits must be finite.");
  }

  double startTime = inputTrajectory.getStartTime();
  /*
  auto dynamicPath = detail::convertToDynamicPath(
      inputTrajectory, maxVelocity, maxAcceleration, false);

  auto outputTrajectory
      = detail::convertToSpline(*dynamicPath, startTime, stateSpace);
  return outputTrajectory;
  */
  return nullptr;
}

//==============================================================================
KinodynamicTimer::KinodynamicTimer(
    const Eigen::VectorXd& velocityLimits,
    const Eigen::VectorXd& accelerationLimits)
  : mVelocityLimits{velocityLimits}, mAccelerationLimits{accelerationLimits}
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> KinodynamicTimer::postprocess(
    const aikido::trajectory::Interpolated& inputTraj,
    const aikido::common::RNG& /*rng*/,
    const aikido::constraint::TestablePtr& /*constraint*/)
{
  return computeKinodynamicTiming(
      inputTraj, mVelocityLimits, mAccelerationLimits);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> KinodynamicTimer::postprocess(
    const aikido::trajectory::Spline& inputTraj,
    const aikido::common::RNG& /*rng*/,
    const aikido::constraint::TestablePtr& /*constraint*/)
{
  return computeKinodynamicTiming(
      inputTraj, mVelocityLimits, mAccelerationLimits);
}

const Eigen::VectorXd& KinodynamicTimer::getVelocityLimits() const
{
  return mVelocityLimits;
}
   
const Eigen::VectorXd& KinodynamicTimer::getAccelerationLimits() const
{
  return mAccelerationLimits;
}

void KinodynamicTimer::setVelocityLimits(const Eigen::VectorXd& velocityLimits)
{
  mVelocityLimits = velocityLimits;
}
  
void KinodynamicTimer::setAccelerationLimits(const Eigen::VectorXd& accelerationLimits)
{
  mAccelerationLimits = accelerationLimits;
}

} // namespace kinodynamic
} // namespace planner
} // namespace aikido
