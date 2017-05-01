#ifndef PARABOLIC_UTIL_HPP_
#define PARABOLIC_UTIL_HPP_

#include <cassert>
#include <set>
#include <dart/dart.hpp>
#include <dart/common/StlHelpers.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/util/Spline.hpp>
#include "DynamicPath.h"
#include "ParabolicUtil.hpp"

using Eigen::Vector2d;
using aikido::statespace::CartesianProduct;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::R;
using aikido::statespace::SO2;
using aikido::statespace::StateSpace;
using dart::common::make_unique;

using CubicSplineProblem
    = aikido::util::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

namespace aikido {
namespace planner {
namespace parabolic {

ParabolicRamp::Vector toVector(const Eigen::VectorXd& _x)
{
  ParabolicRamp::Vector output(_x.size());

  for (int i = 0; i < _x.size(); ++i)
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
    ParabolicRamp::DynamicPath& _path,
    double _t,
    Eigen::VectorXd& _position,
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
  // TODO(JS): Generalize Rn<N> for arbitrary N.
  if (dynamic_cast<const R<0>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<1>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<2>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<3>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<4>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<5>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const R<6>*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const SO2*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (auto space = dynamic_cast<const CartesianProduct*>(_stateSpace))
  {
    for (size_t isubspace = 0; isubspace < space->getNumSubspaces();
         ++isubspace)
    {
      if (!checkStateSpace(space->getSubspace<>(isubspace).get()))
        return false;
    }
    return true;
  }
  else
  {
    return false;
  }
}

aikido::trajectory::Spline* convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory)
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

  auto outputTrajectory = new aikido::trajectory::Spline(
      stateSpace, _inputTrajectory.getStartTime());

  Eigen::VectorXd currentVec, nextVec;

  for (size_t iwaypoint = 0; iwaypoint < numWaypoints-1; ++iwaypoint)
  {
    const auto currentState = _inputTrajectory.getWaypoint(iwaypoint);
    double currentTime = _inputTrajectory.getWaypointTime(iwaypoint);
    const auto nextState = _inputTrajectory.getWaypoint(iwaypoint+1);
    double nextTime = _inputTrajectory.getWaypointTime(iwaypoint+1);

    stateSpace->logMap(currentState, currentVec);
    stateSpace->logMap(nextState, nextVec);

    // Compute the spline coefficients for this segment of the trajectory. Each
    // segment is expressed in the tangent space of the previous segment.
    // TODO: We should apply the adjoint transform to velocity.
    CubicSplineProblem problem(Vector2d(0, nextTime - currentTime), 4, dimension);
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

aikido::trajectory::Spline* convertToSpline(
        ParabolicRamp::DynamicPath& _inputPath,
        aikido::statespace::StateSpacePtr _stateSpace)
{
  const auto dimension = _stateSpace->getDimension();
  //const auto numWaypoints = _inputTrajectory->getNumWaypoints();

  // Construct a list of all ramp transition points.
  double t = 0.;
  std::set<double> transitionTimes;
  transitionTimes.insert(t);

  for (const auto& rampNd : _inputPath.ramps)
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
  evaluateAtTime(_inputPath, timePrev, positionPrev, velocityPrev);

  // TODO: assume _inputPath start time is 0
  auto _outputTrajectory = new aikido::trajectory::Spline(_stateSpace,
                                                          timePrev + 0.0);
  auto segmentStartState = _stateSpace->createState();
  auto relativeState = _stateSpace->createState();
  auto startState = _stateSpace->createState();
  ParabolicRamp::Vector startVec;
  _inputPath.Evaluate(0,startVec);
  _stateSpace->expMap( toEigen( startVec ), startState );

  for (const auto timeCurr : transitionTimes)
  {
    Eigen::VectorXd positionCurr, velocityCurr;
    evaluateAtTime(_inputPath, timeCurr, positionCurr, velocityCurr);

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
    _stateSpace->expMap(positionPrev, relativeState);
    _stateSpace->compose(startState, relativeState, segmentStartState);

    // Add the ramp to the output trajectory.
    assert(spline.getCoefficients().size() == 1);
    const auto& coefficients = spline.getCoefficients().front();
    _outputTrajectory->addSegment(
      coefficients, timeCurr - timePrev, segmentStartState);

    timePrev = timeCurr;
    positionPrev = positionCurr;
    velocityPrev = velocityCurr;
  }

  return _outputTrajectory;
}

void convertToDynamicPath(aikido::trajectory::Spline* _inputTrajectory,
                          ParabolicRamp::DynamicPath& _outputPath)
{
  const auto stateSpace = _inputTrajectory->getStateSpace();
  //const auto dimension = stateSpace->getDimension();
  const auto numWaypoints = _inputTrajectory->getNumWaypoints();

  std::vector<ParabolicRamp::Vector> milestones;
  milestones.reserve(numWaypoints);

  const auto startState = _inputTrajectory->getWaypoint(0);
  const auto startStateInverse = stateSpace->createState();
  stateSpace->getInverse(startState, startStateInverse);

  const auto relativeState = stateSpace->createState();
  Eigen::VectorXd tangentVector;

  for (size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint)
  {
     const auto currentState = _inputTrajectory->getWaypoint(iwaypoint);
     stateSpace->compose(startStateInverse, currentState, relativeState);
     stateSpace->logMap(relativeState, tangentVector);
     milestones.emplace_back(toVector(tangentVector));
  }

  _outputPath.SetMilestones(milestones);
}

} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // PARABOLIC_UTIL_HPP_
