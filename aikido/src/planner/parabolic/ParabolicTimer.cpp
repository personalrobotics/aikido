#include <cassert>
#include <set>
#include <aikido/util/Spline.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <dart/common/StlHelpers.h>
#include "DynamicPath.h"

using Eigen::Vector2d;
using aikido::statespace::CartesianProduct;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::Rn;
using aikido::statespace::SO2;
using aikido::statespace::StateSpace;
using dart::common::make_unique;

using CubicSplineProblem
  = aikido::util::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

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

bool checkStateSpace(const statespace::StateSpace* _stateSpace)
{
  if (dynamic_cast<const Rn*>(_stateSpace) != nullptr)
    return true;
  else if (dynamic_cast<const SO2*>(_stateSpace) != nullptr)
    return true;
  else if (auto space = dynamic_cast<const CartesianProduct*>(_stateSpace))
  {
    for (size_t isubspace = 0; isubspace < space->getNumSubspaces(); ++isubspace)
    {
      if (!checkStateSpace(space->getSubspace<>(isubspace).get()))
        return false;
    }
    return true;
  }
  else
    return false;
}

} // namespace

std::unique_ptr<trajectory::Spline> computeParabolicTiming(
  const trajectory::Interpolated& _inputTrajectory,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _maxAcceleration)
{
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

  if (_maxVelocity.size() != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (_maxAcceleration.size() != dimension)
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

  // Convert the Interpolated to the internal data structure by
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
  double t = 0.;
  std::set<double> transitionTimes;
  transitionTimes.insert(t);

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

  auto outputTrajectory = make_unique<trajectory::Spline>(
    stateSpace, timePrev + _inputTrajectory.getStartTime());
  auto segmentStartState = stateSpace->createState();

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

    // Forward-integrate the trajectory from the start in one step. This should
    // prevent accumulation of numerical error over long trajectories. Note
    // that this is only possible in real vector spaces and SO(2).
    stateSpace->expMap(positionPrev, relativeState);
    stateSpace->compose(startState, relativeState, segmentStartState);

    // Add the ramp to the output trajectory.
    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();
    outputTrajectory->addSegment(
      coefficients, timeCurr - timePrev, segmentStartState);

    timePrev = timeCurr;
    positionPrev = positionCurr;
    velocityPrev = velocityCurr;
  }

  return outputTrajectory;
}

} // namespace parabolic
} // namespace planner
} // namespace aikido
