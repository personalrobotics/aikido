#include "aikido/trajectory/util.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>
#include <dart/common/StlHelpers.hpp>
#include "aikido/common/Spline.hpp"
#include "aikido/common/StepSequence.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/statespace/CartesianProduct.hpp"
#include "aikido/statespace/Rn.hpp"
#include "aikido/statespace/SO2.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

using aikido::statespace::R;
using aikido::statespace::R1;
using aikido::statespace::SO2;
using aikido::statespace::CartesianProduct;
using aikido::statespace::ConstStateSpacePtr;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::StateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using dart::common::make_unique;

using Eigen::Vector2d;
using LinearSplineProblem
    = aikido::common::SplineProblem<double, int, 2, Eigen::Dynamic, 2>;
using CubicSplineProblem = aikido::common::
    SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

namespace aikido {
namespace trajectory {

namespace {

bool checkStateSpace(const statespace::StateSpace* _stateSpace)
{
  // Only supports single-DOF joint spaces, namely R1 and SO2.
  if (dynamic_cast<const R1*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (dynamic_cast<const SO2*>(_stateSpace) != nullptr)
  {
    return true;
  }
  else if (auto space = dynamic_cast<const CartesianProduct*>(_stateSpace))
  {
    for (std::size_t isubspace = 0; isubspace < space->getNumSubspaces();
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

} // namespace

//==============================================================================
UniqueSplinePtr convertToSpline(const Interpolated& inputTrajectory)
{
  using statespace::GeodesicInterpolator;

  if (!std::dynamic_pointer_cast<const GeodesicInterpolator>(
          inputTrajectory.getInterpolator()))
  {
    throw std::invalid_argument(
        "The interpolator of inputTrajectory should be a GeodesicInterpolator");
  }

  const auto stateSpace = inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();
  const auto numWaypoints = inputTrajectory.getNumWaypoints();

  if (!checkStateSpace(stateSpace.get()))
    throw std::invalid_argument(
        "convertToSpline only supports Rn and SO2 joint spaces");

  if (numWaypoints == 0)
    throw std::invalid_argument("Trajectory is empty.");

  auto outputTrajectory
      = make_unique<Spline>(stateSpace, inputTrajectory.getStartTime());

  Eigen::VectorXd currentVec, nextVec;
  for (std::size_t iwaypoint = 0; iwaypoint < numWaypoints - 1; ++iwaypoint)
  {
    const auto currentState = inputTrajectory.getWaypoint(iwaypoint);
    double currentTime = inputTrajectory.getWaypointTime(iwaypoint);
    const auto nextState = inputTrajectory.getWaypoint(iwaypoint + 1);
    double nextTime = inputTrajectory.getWaypointTime(iwaypoint + 1);

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

//==============================================================================
UniqueInterpolatedPtr convertToInterpolated(
    const Spline& traj, statespace::ConstInterpolatorPtr interpolator)
{
  auto stateSpace = traj.getStateSpace();
  auto outputTrajectory
      = make_unique<Interpolated>(stateSpace, std::move(interpolator));

  auto state = stateSpace->createState();
  for (std::size_t i = 0; i < traj.getNumWaypoints(); ++i)
  {
    traj.getWaypoint(i, state);
    const double t = traj.getWaypointTime(i);
    outputTrajectory->addWaypoint(t, state);
  }

  return outputTrajectory;
}

//==============================================================================
UniqueInterpolatedPtr concatenate(
    const Interpolated& traj1, const Interpolated& traj2)
{
  if (traj1.getStateSpace() != traj2.getStateSpace())
    throw std::runtime_error("State space mismatch");

  if (traj1.getInterpolator() != traj2.getInterpolator())
    throw std::runtime_error("Interpolator mismatch");

  auto outputTrajectory = make_unique<Interpolated>(
      traj1.getStateSpace(), traj1.getInterpolator());
  if (traj1.getNumWaypoints() > 1u)
  {
    for (std::size_t i = 0u; i < traj1.getNumWaypoints() - 1u; ++i)
    {
      outputTrajectory->addWaypoint(
          traj1.getWaypointTime(i), traj1.getWaypoint(i));
    }
  }

  const double offset = traj1.getEndTime() - traj2.getStartTime();
  for (std::size_t i = 0u; i < traj2.getNumWaypoints(); ++i)
  {
    outputTrajectory->addWaypoint(
        traj2.getWaypointTime(i) + offset, traj2.getWaypoint(i));
  }

  return outputTrajectory;
}

//==============================================================================
UniqueSplinePtr concatenate(const Spline& traj1, const Spline& traj2)
{
  if (traj1.getStateSpace() != traj2.getStateSpace())
    throw std::runtime_error("State space mismatch");

  const auto& stateSpace = traj1.getStateSpace();
  statespace::ConstInterpolatorPtr interpolator
      = std::make_shared<statespace::GeodesicInterpolator>(stateSpace);
  auto interpolated1 = convertToInterpolated(traj1, interpolator);
  auto interpolated2 = convertToInterpolated(traj2, interpolator);
  auto concatenatedInterpolated = concatenate(*interpolated1, *interpolated2);

  return convertToSpline(*concatenatedInterpolated);
}

//==============================================================================
double findTimeOfClosestStateOnTrajectory(
    const Trajectory& traj,
    const Eigen::VectorXd& referenceState,
    double timeStep)
{
  auto stateSpace = traj.getStateSpace();
  const std::size_t configSize
      = static_cast<std::size_t>(referenceState.size());
  if (configSize != stateSpace->getDimension())
    throw std::runtime_error("Dimension mismatch");

  double findTime = traj.getStartTime();
  double minDist = std::numeric_limits<double>::max();

  const common::StepSequence sequence(
      timeStep, true, true, traj.getStartTime(), traj.getEndTime());

  auto currState = stateSpace->createState();
  Eigen::VectorXd currPos(stateSpace->getDimension());
  for (const double currTime : sequence)
  {
    traj.evaluate(currTime, currState);
    stateSpace->logMap(currState, currPos);

    const double currDist = (referenceState - currPos).norm();
    if (currDist < minDist)
    {
      findTime = currTime;
      minDist = currDist;
    }
  }

  return findTime;
}

//==============================================================================
UniqueSplinePtr createPartialTrajectory(
    const Spline& traj, double partialStartTime)
{
  if (partialStartTime < traj.getStartTime()
      || partialStartTime > traj.getEndTime())
  {
    throw std::runtime_error("Wrong partial start time");
  }

  const auto stateSpace = traj.getStateSpace();
  const int dimension = static_cast<int>(stateSpace->getDimension());
  auto outputTrajectory = make_unique<Spline>(stateSpace, traj.getStartTime());

  double currSegmentStartTime = traj.getStartTime();
  double currSegmentEndTime = currSegmentStartTime;
  std::size_t currSegmentIdx = 0u;

  const auto segmentStartState = stateSpace->createState();
  const auto segmentEndState = stateSpace->createState();
  Eigen::VectorXd segStartPos(dimension);
  Eigen::VectorXd segEndPos(dimension);
  Eigen::VectorXd segStartVel(dimension);
  Eigen::VectorXd segEndVel(dimension);
  const Eigen::VectorXd zeroPos = Eigen::VectorXd::Zero(dimension);
  traj.evaluate(partialStartTime, segmentStartState);
  stateSpace->logMap(segmentStartState, segStartPos);
  traj.evaluateDerivative(partialStartTime, 1, segStartVel);

  // Find a segment of original trajectory that includes partialStartTime
  while (currSegmentIdx < traj.getNumSegments())
  {
    currSegmentEndTime += traj.getSegmentDuration(currSegmentIdx);
    if (currSegmentStartTime <= partialStartTime
        && partialStartTime <= currSegmentEndTime)
    {
      // create new segment
      traj.evaluate(currSegmentEndTime, segmentEndState);
      stateSpace->logMap(segmentEndState, segEndPos);
      traj.evaluateDerivative(currSegmentEndTime, 1, segEndVel);

      const double segmentDuration = currSegmentEndTime - partialStartTime;

      if (segmentDuration > 0.0)
      {
        CubicSplineProblem problem(Vector2d{0., segmentDuration}, 4, dimension);
        problem.addConstantConstraint(0, 0, zeroPos);
        problem.addConstantConstraint(0, 1, segStartVel);
        problem.addConstantConstraint(1, 0, segEndPos - segStartPos);
        problem.addConstantConstraint(1, 1, segEndVel);
        const auto solution = problem.fit();
        const auto coefficients = solution.getCoefficients().front();
        outputTrajectory->addSegment(
            coefficients, segmentDuration, segmentStartState);
      }
      break;
    }

    currSegmentIdx++;
    currSegmentStartTime = currSegmentEndTime;
  }

  for (std::size_t i = currSegmentIdx + 1u; i < traj.getNumSegments(); ++i)
  {
    outputTrajectory->addSegment(
        traj.getSegmentCoefficients(i),
        traj.getSegmentDuration(i),
        traj.getSegmentStartState(i));
  }

  return outputTrajectory;
}

//==============================================================================
UniqueInterpolatedPtr toR1JointTrajectory(const Interpolated& trajectory)
{
  if (!checkStateSpace(trajectory.getStateSpace().get()))
    throw std::invalid_argument(
        "toR1JointTrajectory only supports R1 and SO2 joint spaces");

  auto interpolator = std::dynamic_pointer_cast<const GeodesicInterpolator>(
      trajectory.getInterpolator());
  if (!interpolator)
    throw std::invalid_argument(
        "The interpolator of trajectory should be a GeodesicInterpolator");

  // Create new trajectory space.
  auto space = trajectory.getStateSpace();
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < space->getDimension(); ++i)
    subspaces.emplace_back(std::make_shared<const R1>());

  auto rSpace = std::make_shared<CartesianProduct>(subspaces);
  auto rInterpolator = std::make_shared<GeodesicInterpolator>(rSpace);
  auto rTrajectory = make_unique<Interpolated>(rSpace, rInterpolator);

  Eigen::VectorXd sourceVector(space->getDimension());
  auto sourceState = rSpace->createState();

  // Add the first waypoint
  space->logMap(trajectory.getWaypoint(0), sourceVector);
  rSpace->expMap(sourceVector, sourceState);
  rTrajectory->addWaypoint(0, sourceState);

  auto tangentState = rSpace->createState();
  auto targetState = rSpace->createState();

  // Add the remaining waypoints
  for (std::size_t i = 0; i < trajectory.getNumWaypoints() - 1; ++i)
  {
    const auto tangentVector = interpolator->getTangentVector(
        trajectory.getWaypoint(i), trajectory.getWaypoint(i + 1));

    rSpace->expMap(sourceVector, sourceState);
    rSpace->expMap(tangentVector, tangentState);
    rSpace->compose(sourceState, tangentState, targetState);

    rTrajectory->addWaypoint(i + 1, targetState);
    rSpace->logMap(targetState, sourceVector);
  }

  return rTrajectory;
}

//==============================================================================
UniqueSplinePtr toR1JointTrajectory(const Spline& trajectory)
{
  if (!checkStateSpace(trajectory.getStateSpace().get()))
    throw std::invalid_argument(
        "toR1JointTrajectory only supports R1 and SO2 joint spaces");

  auto space = trajectory.getStateSpace();
  aikido::statespace::ConstInterpolatorPtr interpolator
      = std::make_shared<statespace::GeodesicInterpolator>(space);

  ConstInterpolatedPtr interpolatedTrajectory
      = std::move(convertToInterpolated(trajectory, interpolator));
  auto r1JointTrajectory = toR1JointTrajectory(*interpolatedTrajectory);
  auto splineTrajectory = convertToSpline(*r1JointTrajectory);

  return splineTrajectory;
}

} // namespace trajectory
} // namespace aikido
