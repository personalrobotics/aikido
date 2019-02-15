#include "ParabolicUtil.hpp"

#include <cassert>
#include <set>

#include <dart/common/StlHelpers.hpp>
#include <dart/dart.hpp>
#include "aikido/common/Spline.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/util.hpp"

#include "DynamicPath.h"

using Eigen::Vector2d;
using aikido::statespace::StateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::trajectory::toR1JointTrajectory;
using aikido::trajectory::ConstInterpolatedPtr;
using aikido::trajectory::ConstSplinePtr;
using dart::common::make_unique;

using CubicSplineProblem
    = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

namespace aikido {
namespace planner {
namespace parabolic {
namespace detail {

//==============================================================================
ParabolicRamp::Vector toVector(const Eigen::VectorXd& _x)
{
  ParabolicRamp::Vector output(_x.size());

  for (int i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

//==============================================================================
Eigen::VectorXd toEigen(const ParabolicRamp::Vector& _x)
{
  Eigen::VectorXd output(_x.size());

  for (std::size_t i = 0; i < _x.size(); ++i)
    output[i] = _x[i];

  return output;
}

//==============================================================================
void evaluateAtTime(
    const ParabolicRamp::DynamicPath& _path,
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

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const ParabolicRamp::DynamicPath& _inputPath,
    double _startTime,
    statespace::ConstStateSpacePtr _stateSpace)
{
  const auto dimension = _stateSpace->getDimension();

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

  auto _outputTrajectory = make_unique<aikido::trajectory::Spline>(
      _stateSpace, timePrev + _startTime);
  auto segmentStartState = _stateSpace->createState();

  for (const auto timeCurr : transitionTimes)
  {
    Eigen::VectorXd positionCurr, velocityCurr;
    evaluateAtTime(_inputPath, timeCurr, positionCurr, velocityCurr);

    CubicSplineProblem problem(Vector2d(0, timeCurr - timePrev), 4, dimension);
    problem.addConstantConstraint(0, 0, Eigen::VectorXd::Zero(dimension));
    problem.addConstantConstraint(0, 1, velocityPrev);
    problem.addConstantConstraint(1, 0, positionCurr - positionPrev);
    problem.addConstantConstraint(1, 1, velocityCurr);
    const auto spline = problem.fit();

    _stateSpace->expMap(positionPrev, segmentStartState);

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

//==============================================================================
std::unique_ptr<ParabolicRamp::DynamicPath> convertToDynamicPath(
    const aikido::trajectory::Spline& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    bool _preserveWaypointVelocity)
{
  auto stateSpace = _inputTrajectory.getStateSpace();
  auto metaSkeletonStateSpace
      = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(stateSpace);

  const aikido::trajectory::Spline* trajectory = &_inputTrajectory;

  ConstSplinePtr r1Trajectory;
  if (metaSkeletonStateSpace)
  {
    r1Trajectory = toR1JointTrajectory(_inputTrajectory);
    stateSpace = r1Trajectory->getStateSpace();
    trajectory = r1Trajectory.get();
  }

  const auto numWaypoints = _inputTrajectory.getNumWaypoints();

  // TODO(brian): debug
  // auto trajectory = toR1JointTrajectory(_inputTrajectory);
  // auto stateSpace = trajectory->getStateSpace();
  // const auto numWaypoints = trajectory->getNumWaypoints();

  std::vector<ParabolicRamp::Vector> milestones;
  std::vector<ParabolicRamp::Vector> velocities;
  milestones.reserve(numWaypoints);
  velocities.reserve(numWaypoints);

  Eigen::VectorXd tangentVector, currVec;

  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint)
  {
    auto currentState = stateSpace->createState();
    trajectory->getWaypoint(iwaypoint, currentState);

    stateSpace->logMap(currentState, currVec);
    milestones.emplace_back(toVector(currVec));

    trajectory->getWaypointDerivative(iwaypoint, 1, tangentVector);
    velocities.emplace_back(toVector(tangentVector));
  }

  auto outputPath = make_unique<ParabolicRamp::DynamicPath>();
  outputPath->Init(toVector(_maxVelocity), toVector(_maxAcceleration));
  if (_preserveWaypointVelocity)
  {
    outputPath->SetMilestones(milestones, velocities);
  }
  else
  {
    outputPath->SetMilestones(milestones);
  }
  if (!outputPath->IsValid())
    throw std::runtime_error("Converted DynamicPath is not valid");
  return outputPath;
}

//==============================================================================
std::unique_ptr<ParabolicRamp::DynamicPath> convertToDynamicPath(
    const aikido::trajectory::Interpolated& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration)
{
  auto stateSpace = _inputTrajectory.getStateSpace();
  auto metaSkeletonStateSpace
      = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(stateSpace);

  const auto numWaypoints = _inputTrajectory.getNumWaypoints();

  const aikido::trajectory::Interpolated* trajectory = &_inputTrajectory;

  ConstInterpolatedPtr r1Trajectory;
  if (metaSkeletonStateSpace)
  {
    r1Trajectory = toR1JointTrajectory(_inputTrajectory);
    stateSpace = r1Trajectory->getStateSpace();
    trajectory = r1Trajectory.get();
  }

  // TODO(brian): debug
  // auto trajectory = toR1JointTrajectory(_inputTrajectory);
  // auto stateSpace = trajectory->getStateSpace();
  // const auto numWaypoints = trajectory->getNumWaypoints();

  std::vector<ParabolicRamp::Vector> milestones;
  std::vector<ParabolicRamp::Vector> velocities;
  milestones.reserve(numWaypoints);
  velocities.reserve(numWaypoints);

  Eigen::VectorXd currVec;

  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint)
  {
    auto currentState = trajectory->getWaypoint(iwaypoint);
    stateSpace->logMap(currentState, currVec);
    milestones.emplace_back(toVector(currVec));
  }

  auto outputPath = make_unique<ParabolicRamp::DynamicPath>();
  outputPath->Init(toVector(_maxVelocity), toVector(_maxAcceleration));
  outputPath->SetMilestones(milestones);
  if (!outputPath->IsValid())
    throw std::runtime_error("Converted DynamicPath is not valid");
  return outputPath;
}

} // namespace detail
} // namespace parabolic
} // namespace planner
} // namespace aikido
