#include <algorithm>
#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>
#include <dart/common/StlHelpers.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/common/StepSequence.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/util.hpp>

namespace po = boost::program_options;

namespace aikido {
namespace trajectory {

//==============================================================================
double findClosetStateOnTrajectory(
    const aikido::trajectory::Trajectory* traj,
    const Eigen::VectorXd& config,
    double timeStep)
{
  if (traj == nullptr)
    throw std::runtime_error("Traj is nullptr");
  auto stateSpace = traj->getStateSpace();
  std::size_t configSize = config.size();
  if (configSize != stateSpace->getDimension())
    throw std::runtime_error("Dimension mismatch");

  double findTime = traj->getStartTime();
  double minDist = std::numeric_limits<double>::max();

  aikido::common::StepSequence sequence(
      timeStep, true, true, traj->getStartTime(), traj->getEndTime());

  auto currState = stateSpace->createState();
  Eigen::VectorXd currPos(stateSpace->getDimension());
  for (std::size_t i = 0; i < sequence.getLength(); i++)
  {
    double currTime = sequence[i];
    traj->evaluate(currTime, currState);
    stateSpace->logMap(currState, currPos);

    double currDist = (config - currPos).norm();
    if (currDist < minDist)
    {
      findTime = currTime;
      minDist = currDist;
    }
  }
  std::cout << "findClosestStateOnTrajectory minDist: " << minDist << std::endl;

  return findTime;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> createPartialTrajectory(
    const aikido::trajectory::Spline& traj, double partialStartTime)
{
  if (partialStartTime < traj.getStartTime()
      || partialStartTime > traj.getEndTime())
    throw std::runtime_error("Wrong partial start time");

  using dart::common::make_unique;
  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

  auto stateSpace = traj.getStateSpace();
  std::size_t dimension = stateSpace->getDimension();
  auto outputTrajectory = make_unique<aikido::trajectory::Spline>(
      stateSpace, traj.getStartTime());

  double currSegmentStartTime = traj.getStartTime();
  double currSegmentEndTime = currSegmentStartTime;
  std::size_t currSegmentIdx = 0;

  auto segmentStartState = stateSpace->createState();
  auto segmentEndState = stateSpace->createState();
  Eigen::VectorXd segStartPos(dimension), segEndPos(dimension),
      segStartVel(dimension), segEndVel(dimension);
  const Eigen::VectorXd zeroPos = Eigen::VectorXd::Zero(dimension);
  traj.evaluate(partialStartTime, segmentStartState);
  stateSpace->logMap(segmentStartState, segStartPos);
  traj.evaluateDerivative(partialStartTime, 1, segStartVel);

  while (currSegmentIdx < traj.getNumSegments())
  {
    currSegmentEndTime += traj.getSegmentDuration(currSegmentIdx);
    if (partialStartTime >= currSegmentStartTime
        && partialStartTime <= currSegmentEndTime)
    {
      std::cout << "FIND " << partialStartTime << " IN " << currSegmentIdx
                << "-th [" << currSegmentStartTime;
      std::cout << " , " << currSegmentEndTime << "]" << std::endl;
      // create new segment
      traj.evaluate(currSegmentEndTime, segmentEndState);
      stateSpace->logMap(segmentEndState, segEndPos);
      traj.evaluateDerivative(currSegmentEndTime, 1, segEndVel);

      double segmentDuration = currSegmentEndTime - partialStartTime;

      if (segmentDuration > 0.0)
      {
        CubicSplineProblem problem(
            Eigen::Vector2d{0., segmentDuration}, 4, dimension);
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

  for (std::size_t i = currSegmentIdx + 1; i < traj.getNumSegments(); i++)
  {
    // std::cout << "CONTINUE ADDING " << i << "-th SEGMENT" << std::endl;
    outputTrajectory->addSegment(
        traj.getSegmentCoefficients(i),
        traj.getSegmentDuration(i),
        traj.getSegmentState(i));
  }

  return outputTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Interpolated> convertToInterpolated(
    const aikido::trajectory::Spline& traj)
{
  using dart::common::make_unique;
  auto stateSpace = traj.getStateSpace();

  auto interpolator
      = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);

  auto outputTrajectory
      = make_unique<aikido::trajectory::Interpolated>(stateSpace, interpolator);

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
std::unique_ptr<aikido::trajectory::Interpolated> concatenate(
    const aikido::trajectory::Interpolated& traj1,
    const aikido::trajectory::Interpolated& traj2)
{
  if (traj1.getStateSpace()->getDimension()
      != traj2.getStateSpace()->getDimension())
    throw std::runtime_error("Dimension mismatch");

  using dart::common::make_unique;
  auto stateSpace = traj1.getStateSpace();

  auto outputTrajectory = make_unique<aikido::trajectory::Interpolated>(
      stateSpace, traj1.getInterpolator());

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
std::unique_ptr<aikido::trajectory::Spline> concatenate(
    const aikido::trajectory::Spline& traj1,
    const aikido::trajectory::Spline& traj2)
{
  auto interpolated1 = convertToInterpolated(traj1);
  auto interpolated2 = convertToInterpolated(traj2);
  auto concatenatedInterpolated = concatenate(*interpolated1, *interpolated2);
  return aikido::planner::parabolic::convertToSpline(*concatenatedInterpolated);
}

} // namespace trajectory
} // namespace aikido
