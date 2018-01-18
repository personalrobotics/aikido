#include <cassert>
#include <set>
#include <dart/common/StlHelpers.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include "DynamicPath.h"
#include "ParabolicUtil.hpp"

using Eigen::Vector2d;
using dart::common::make_unique;

using LinearSplineProblem
    = aikido::common::SplineProblem<double, int, 2, Eigen::Dynamic, 2>;

namespace aikido {
namespace planner {
namespace parabolic {

std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory)
{
  using aikido::statespace::GeodesicInterpolator;

  if (nullptr == dynamic_cast<GeodesicInterpolator*>(
                     _inputTrajectory.getInterpolator().get()))
  {
    throw std::invalid_argument(
        "The interpolator of _inputTrajectory should be a "
        "GeodesicInterpolator");
  }

  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();
  const auto numWaypoints = _inputTrajectory.getNumWaypoints();

  if (!detail::checkStateSpace(stateSpace.get()))
    throw std::invalid_argument(
        "computeParabolicTiming only supports Rn, "
        "SO2, and CartesianProducts consisting of those types.");

  const auto interpolator = _inputTrajectory.getInterpolator();
  if (dynamic_cast<const GeodesicInterpolator*>(interpolator.get()) == nullptr)
    throw std::invalid_argument(
        "computeParabolicTiming only supports geodesic interpolation.");

  if (numWaypoints == 0)
    throw std::invalid_argument("Trajectory is empty.");

  auto outputTrajectory = make_unique<aikido::trajectory::Spline>(
      stateSpace, _inputTrajectory.getStartTime());

  Eigen::VectorXd currentVec, nextVec;
  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints - 1; ++iwaypoint)
  {
    const auto currentState = _inputTrajectory.getWaypoint(iwaypoint);
    double currentTime = _inputTrajectory.getWaypointTime(iwaypoint);
    const auto nextState = _inputTrajectory.getWaypoint(iwaypoint + 1);
    double nextTime = _inputTrajectory.getWaypointTime(iwaypoint + 1);

    stateSpace->logMap(currentState, currentVec);
    stateSpace->logMap(nextState, nextVec);

    // Compute the spline coefficients for this segment of the trajectory.
    LinearSplineProblem problem(
        Vector2d(0, nextTime - currentTime), 2, dimension);
    problem.addConstantConstraint(0, 0, Eigen::VectorXd::Zero(dimension));
    problem.addConstantConstraint(1, 0, nextVec - currentVec);
    const auto spline = problem.fit();

    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();
    outputTrajectory->addSegment(
        coefficients, nextTime - currentTime, currentState);
  }

  return outputTrajectory;
}

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

RetimeTrajectoryPostProcessor::RetimeTrajectoryPostProcessor(
    aikido::statespace::StateSpacePtr _space,
    const Eigen::VectorXd& _velocityLimits,
    const Eigen::VectorXd& _accelerationLimits)
  : mSpace{std::move(_space)}
  , mVelocityLimits{_velocityLimits}
  , mAccelerationLimits{_accelerationLimits}
{
  // Do nothing
}

std::unique_ptr<aikido::trajectory::Spline>
RetimeTrajectoryPostProcessor::postprocess(
    const aikido::trajectory::InterpolatedPtr& _inputTraj,
    const aikido::common::RNG* /*_rng*/)
{
  if (!_inputTraj)
    throw std::invalid_argument(
        "Passed nullptr _inputTraj to "
        "RetimeTrajectoryPostProcessor::postprocess");

  return computeParabolicTiming(
      *_inputTraj, mVelocityLimits, mAccelerationLimits);
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
