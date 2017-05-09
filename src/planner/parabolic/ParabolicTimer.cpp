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

std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory)
{
  using aikido::statespace::GeodesicInterpolator;

  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();
  const auto numWaypoints = _inputTrajectory.getNumWaypoints();

  if (!checkStateSpace(stateSpace.get()))
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

  Eigen::VectorXd currentVec, nextVec, currentVelocity, nextVelocity;

  for (size_t iwaypoint = 0; iwaypoint < numWaypoints - 1; ++iwaypoint)
  {
    const auto currentState = _inputTrajectory.getWaypoint(iwaypoint);
    double currentTime = _inputTrajectory.getWaypointTime(iwaypoint);
    const auto nextState = _inputTrajectory.getWaypoint(iwaypoint + 1);
    double nextTime = _inputTrajectory.getWaypointTime(iwaypoint + 1);

    stateSpace->logMap(currentState, currentVec);
    stateSpace->logMap(nextState, nextVec);

    _inputTrajectory.evaluateDerivative(currentTime, 1, currentVelocity);
    _inputTrajectory.evaluateDerivative(nextTime, 1, nextVelocity);

    // Compute the spline coefficients for this segment of the trajectory.
    CubicSplineProblem problem(
        Vector2d(0, nextTime - currentTime), 4, dimension);
    problem.addConstantConstraint(0, 0, Eigen::VectorXd::Zero(dimension));
    problem.addConstantConstraint(0, 1, currentVelocity);
    problem.addConstantConstraint(1, 0, nextVec - currentVec);
    problem.addConstantConstraint(1, 1, nextVelocity);
    const auto spline = problem.fit();

    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();
    outputTrajectory->addSegment(
        coefficients, nextTime - currentTime, currentState);
  }

  return outputTrajectory;
}

std::unique_ptr<aikido::trajectory::Spline> computeParabolicTiming(
    const aikido::trajectory::Spline& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
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

  double startTime = _inputTrajectory.getStartTime();
  auto dynamicPath = convertToDynamicPath(
      _inputTrajectory, _maxVelocity, _maxAcceleration, false);

  auto outputTrajectory
      = convertToSpline(*dynamicPath.get(), startTime, stateSpace);
  return outputTrajectory;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
