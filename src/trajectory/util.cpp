#include <algorithm>
#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>
#include <dart/common/StlHelpers.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/common/StepSequence.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/util.hpp>

using aikido::statespace::R;
using aikido::statespace::SO2;
using aikido::statespace::CartesianProduct;
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
  else if (dynamic_cast<const R<Eigen::Dynamic>*>(_stateSpace) != nullptr)
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

} // (anonymous) namespace

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
        "computeParabolicTiming only supports Rn, "
        "SO2, and CartesianProducts consisting of those types.");

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
double findTimeOfClosetStateOnTrajectory(
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

  for (std::size_t i = currSegmentIdx + 1u; i < traj.getNumSegments(); i++)
  {
    outputTrajectory->addSegment(
        traj.getSegmentCoefficients(i),
        traj.getSegmentDuration(i),
        traj.getSegmentState(i));
  }

  return outputTrajectory;
}

//==============================================================================
UniqueInterpolatedPtr convertToInterpolated(
    const Spline& traj, statespace::ConstInterpolatorPtr& interpolator)
{
  auto stateSpace = traj.getStateSpace();
  auto outputTrajectory = make_unique<Interpolated>(stateSpace, interpolator);

  auto state = stateSpace->createState();
  double t = 0.0;
  for (std::size_t i = 0; i < traj.getNumWaypoints(); i++)
  {
    traj.getWaypoint(i, state);
    t = traj.getWaypointTime(i);
    outputTrajectory->addWaypoint(t, state);
  }

  return outputTrajectory;
}

//==============================================================================
UniqueInterpolatedPtr concatenate(
    const Interpolated& traj1, const Interpolated& traj2)
{
  if (traj1.getStateSpace()->getDimension()
      != traj2.getStateSpace()->getDimension())
    throw std::runtime_error("Dimension mismatch");

  auto stateSpace = traj1.getStateSpace();

  auto outputTrajectory
      = make_unique<Interpolated>(stateSpace, traj1.getInterpolator());

  for (std::size_t i = 0; i < traj1.getNumWaypoints() - 1; i++)
  {
    outputTrajectory->addWaypoint(
        traj1.getWaypointTime(i), traj1.getWaypoint(i));
  }
  outputTrajectory->addWaypoint(traj1.getEndTime(), traj2.getWaypoint(0));
  double endTimeOfTraj1 = traj1.getEndTime();
  for (std::size_t i = 1; i < traj2.getNumWaypoints(); i++)
  {
    outputTrajectory->addWaypoint(
        endTimeOfTraj1 + traj2.getWaypointTime(i), traj2.getWaypoint(i));
  }
  return outputTrajectory;
}

//==============================================================================
UniqueSplinePtr concatenate(const Spline& traj1, const Spline& traj2)
{
  if (traj1.getStateSpace() != traj2.getStateSpace())
    throw std::runtime_error("State space mismatch");
  auto stateSpace = traj1.getStateSpace();
  statespace::ConstInterpolatorPtr interpolator
      = std::make_shared<statespace::GeodesicInterpolator>(stateSpace);
  auto interpolated1 = convertToInterpolated(traj1, interpolator);
  auto interpolated2 = convertToInterpolated(traj2, interpolator);
  auto concatenatedInterpolated = concatenate(*interpolated1, *interpolated2);
  return convertToSpline(*concatenatedInterpolated);
}

} // namespace trajectory
} // namespace aikido
