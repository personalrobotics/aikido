#include <sstream>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/util/Spline.hpp>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using SplineTrajectory = aikido::trajectory::Spline;

namespace aikido {
namespace control {
namespace ros {
namespace {

//=============================================================================
void checkVector(
  const std::string& name, const std::vector<double>& values,
  size_t expectedLength, bool isRequired, Eigen::VectorXd* output)
{
  if (values.empty())
  {
    if (isRequired)
    {
      std::stringstream message;
      message << name << " are required.";
      throw std::invalid_argument{message.str()};
    }
  }
  else if (values.size() != expectedLength)
  {
    std::stringstream message;
    message << "Expected " << name << " to be of length " << expectedLength
      << ", got " << values.size() << ".";
    throw std::invalid_argument{message.str()};
  }

  if (output)
    *output = Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
}

//=============================================================================
Eigen::MatrixXd fitPolynomial(
  double currTime,
  const Eigen::VectorXd& currPosition,
  const Eigen::VectorXd& currVelocity,
  const Eigen::VectorXd& currAcceleration,
  double nextTime,
  const Eigen::VectorXd& nextPosition,
  const Eigen::VectorXd& nextVelocity,
  const Eigen::VectorXd& nextAcceleration,
  size_t numCoefficients)
{
  using aikido::util::SplineProblem;

  assert(numCoefficients == 2 || numCoefficients == 4 || numCoefficients == 6);

  const auto numDofs = currPosition.size();
  SplineProblem<> splineProblem(
    Eigen::Vector2d(currTime, nextTime), numCoefficients, numDofs);

  assert(currPosition.size() == numDofs);
  assert(nextPosition.size() == numDofs);
  splineProblem.addConstantConstraint(0, 0, currPosition);
  splineProblem.addConstantConstraint(1, 0, nextPosition);

  if (numCoefficients >= 4)
  {
    assert(currVelocity.size() == numDofs);
    assert(nextVelocity.size() == numDofs);
    splineProblem.addConstantConstraint(0, 1, currVelocity);
    splineProblem.addConstantConstraint(1, 1, nextVelocity);
  }

  if (numCoefficients >= 6)
  {
    assert(currAcceleration.size() == numDofs);
    assert(nextAcceleration.size() == numDofs);
    splineProblem.addConstantConstraint(0, 2, currAcceleration);
    splineProblem.addConstantConstraint(1, 2, nextAcceleration);
  }

  const auto splineSegment = splineProblem.fit();
  return splineSegment.getCoefficients()[0];
}

//=============================================================================
void extractJointTrajectoryPoint(
  const trajectory_msgs::JointTrajectory& trajectory,
  size_t index, size_t numDofs,
  Eigen::VectorXd* positions, bool positionsRequired,
  Eigen::VectorXd* velocities, bool velocitiesRequired,
  Eigen::VectorXd* accelerations, bool accelerationsRequired)
{
  const auto& waypoint = trajectory.points[index];

  try
  {
    checkVector("positions", waypoint.positions, numDofs,
      positionsRequired, positions);
    checkVector("velocities", waypoint.velocities, numDofs,
      velocitiesRequired, velocities);
    checkVector("accelerations", waypoint.accelerations, numDofs,
      accelerationsRequired, accelerations);
  }
  catch (const std::invalid_argument& e)
  {
    std::stringstream message;
    message << "Waypoint " << index << " is invalid: " << e.what();
    throw std::invalid_argument(message.str());
  }
}

} // namespace

//=============================================================================
std::unique_ptr<SplineTrajectory> convertJointTrajectory(
  const std::shared_ptr<MetaSkeletonStateSpace>& space,
  const trajectory_msgs::JointTrajectory& jointTrajectory)
{
  if (!space)
    throw std::invalid_argument{"StateSpace must be non-null."};

  const auto numControlledDofs = space->getNumSubspaces();
  if (jointTrajectory.joint_names.size() != numControlledDofs)
  {
    std::stringstream message;
    message << "Incorrect number of joints: expected "
        << numControlledDofs << ", got "
        << jointTrajectory.joint_names.size() << ".";
    throw std::invalid_argument{message.str()};
  }

  if (jointTrajectory.points.size() < 2)
  {
    throw std::invalid_argument{
      "Trajectory must contain at two or more waypoints."};
  }

  // TODO: Map betwen joint names and indices in the MetaSkeletonStateSpace.

  // Extract the first waypoint to infer the dimensionality of the trajectory.
  Eigen::VectorXd currPosition, currVelocity, currAcceleration;
  extractJointTrajectoryPoint(jointTrajectory, 0, numControlledDofs,
    &currPosition, true, &currVelocity, false, &currAcceleration, false);

  const auto& firstWaypoint = jointTrajectory.points.front();
  auto currTimeFromStart = firstWaypoint.time_from_start.toSec();

  const auto isPositionRequired = true;
  const auto isVelocityRequired = (currVelocity.size() != 0);
  const auto isAccelerationRequired = (currAcceleration.size() != 0);
  if (isAccelerationRequired && !isVelocityRequired)
  {
    throw std::invalid_argument{
      "Velocity is required since acceleration is specified."};
  }

  int numCoefficients;
  if (isAccelerationRequired)
    numCoefficients = 6; // quintic
  else if (isVelocityRequired)
    numCoefficients = 4; // cubic
  else
    numCoefficients = 2; // linear;

  // Convert the ROS trajectory message to an Aikido spline.
  std::unique_ptr<SplineTrajectory> trajectory{new SplineTrajectory{space}};
  auto currState = space->createState();

  const auto& waypoints = jointTrajectory.points;
  for (size_t iwaypoint = 1; iwaypoint < waypoints.size(); ++iwaypoint)
  {
    Eigen::VectorXd nextPosition, nextVelocity, nextAcceleration;
    extractJointTrajectoryPoint(jointTrajectory, iwaypoint, numControlledDofs,
      &nextPosition, isPositionRequired,
      &nextVelocity, isVelocityRequired,
      &nextAcceleration, isAccelerationRequired);

    // Compute spline coefficients for this polynomial segment.
    const auto nextTimeFromStart = waypoints[iwaypoint].time_from_start.toSec();
    const auto segmentDuration = nextTimeFromStart - currTimeFromStart;
    const auto segmentCoefficients = fitPolynomial(
      0., Eigen::VectorXd::Zero(numControlledDofs), currVelocity, currAcceleration,
      segmentDuration, nextPosition - currPosition, nextVelocity, nextAcceleration,
      numCoefficients);

    // Add a segment to the trajectory.
    space->convertPositionsToState(currPosition, currState);
    trajectory->addSegment(segmentCoefficients, segmentDuration, currState);

    // Advance to the next segment.
    currPosition = nextPosition;
    currVelocity = nextVelocity;
    currAcceleration = nextAcceleration;
    currTimeFromStart = nextTimeFromStart;
  }

  return std::move(trajectory);
}

} // namespace ros
} // namespace control
} // namespace aikido
