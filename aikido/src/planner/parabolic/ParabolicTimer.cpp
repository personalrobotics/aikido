#include <cassert>
#include <set>
#include <aikido/path/Spline.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <dart/common/StlHelpers.h>
#include "DynamicPath.h"

using Eigen::Vector2d;
using aikido::statespace::GeodesicInterpolator;
using dart::common::make_unique;

using CubicSplineProblem
  = aikido::path::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

namespace aikido {
namespace planner {
namespace parabolic {
namespace {

ParabolicRamp::Vector toVector(const Eigen::VectorXd& _x)
{
  ParabolicRamp::Vector output(_x.size());

  for (size_t i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

Eigen::VectorXd toEigen(const ParabolicRamp::Vector& _x)
{
  Eigen::VectorXd output(_x.size());

  for (size_t i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

void evaluateAtTime(
  ParabolicRamp::DynamicPath& _path, double _t, Eigen::VectorXd& _position,
  Eigen::VectorXd& _velocity)
{
  ParabolicRamp::Vector positionVector;
  _path.Evaluate(_t, positionVector);
  _position = toEigen(positionVector);

  ParabolicRamp::Vector velocityVector;
  _path.Derivative(_t, velocityVector);
  _velocity = toEigen(velocityVector);
}

} // namespace

std::unique_ptr<path::SplineTrajectory2> computeParabolicTiming(
  const path::PiecewiseLinearTrajectory& _inputTrajectory,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _maxAcceleration)
{
  const auto stateSpace = _inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();
  const auto numWaypoints = _inputTrajectory.getNumWaypoints();

  const auto interpolator = _inputTrajectory.getInterpolator();
  if (dynamic_cast<const GeodesicInterpolator*>(interpolator.get()) == nullptr)
    throw std::invalid_argument(
      "computeParabolicTiming only supports geodesic interpolation.");

  if (numWaypoints == 0)
    throw std::invalid_argument("Trajectory is empty.");

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

  // Convert the PiecewiseLinearTrajectory to the internal data structure by
  // computing the logMap relative to the starting state.
  std::vector<ParabolicRamp::Vector> milestones;
  milestones.reserve(numWaypoints);

  const auto startState = _inputTrajectory.getWaypoint(0);
  const auto startStateInverse = stateSpace->createState();
  stateSpace->getInverse(startState, startStateInverse);

  const auto relativeState = stateSpace->createState();
  Eigen::VectorXd tangentVector; 

  for (size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint)
  {
    const auto currentState = _inputTrajectory.getWaypoint(iwaypoint);
    stateSpace->compose(startStateInverse, currentState, relativeState);
    stateSpace->logMap(relativeState, tangentVector);
    milestones.emplace_back(toVector(tangentVector));
  }

  // Compute the timing of the path.
  ParabolicRamp::DynamicPath dynamicPath;
  // TODO: Should I apply the adjoint transform to these limits?
  dynamicPath.Init(toVector(_maxVelocity), toVector(_maxAcceleration));
  dynamicPath.SetMilestones(milestones);

  // Construct a list of all ramp transition points.
  std::set<double> transitionTimes;
  double t = 0.;

  for (const auto& rampNd : dynamicPath.ramps)
  {
    for (const auto& ramp1d : rampNd.ramps)
    {
      transitionTimes.insert(t + ramp1d.tswitch1);
      transitionTimes.insert(t + ramp1d.tswitch2);
    }

    t += rampNd.endTime;
    transitionTimes.insert(t);
  }

  // Convert the output to a spline with a knot at each transition time.
  assert(!transitionTimes.empty());
  const auto startIt = std::begin(transitionTimes);
  double timePrev = *startIt;
  transitionTimes.erase(startIt);

  Eigen::VectorXd positionPrev, velocityPrev;
  evaluateAtTime(dynamicPath, timePrev, positionPrev, velocityPrev);

  auto outputTrajectory = make_unique<path::SplineTrajectory2>(
    stateSpace, startState, timePrev);

  for (const auto timeCurr : transitionTimes)
  {
    Eigen::VectorXd positionCurr, velocityCurr;
    evaluateAtTime(dynamicPath, timeCurr, positionCurr, velocityCurr);

    // Compute the spline coefficients for this segment of the trajectory. Each
    // segment is expressed in the tangent space of the previous segment.
    // TODO: We should apply the adjoint transform to velocity.
    CubicSplineProblem problem(Vector2d(0, timeCurr - timePrev), 4, dimension);
    problem.addConstantConstraint(0, 0, Eigen::VectorXd::Zero(dimension));
    problem.addConstantConstraint(0, 1, velocityPrev);
    problem.addConstantConstraint(1, 0, positionCurr - positionPrev);
    problem.addConstantConstraint(1, 1, velocityCurr);
    const auto spline = problem.fit();

    // Add the ramp to the output trajectory.
    const auto& coefficients = spline.getCoefficients().front();
    outputTrajectory->addSegment(coefficients, timeCurr - timePrev);

    timePrev = timeCurr;
    positionPrev = positionCurr;
    velocityPrev = velocityCurr;
  }

  return outputTrajectory;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
