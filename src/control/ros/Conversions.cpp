#include <sstream>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/util/Spline.hpp>
#include <dart/dynamics/Joint.hpp>
#include <map>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using SplineTrajectory = aikido::trajectory::Spline;

namespace aikido {
namespace control {
namespace ros {
namespace {

//=============================================================================
void checkVector(
  const std::string& _name, const std::vector<double>& _values,
  size_t _expectedLength, bool _isRequired, Eigen::VectorXd* _output)
{
  if (_values.empty())
  {
    if (_isRequired)
    {
      std::stringstream message;
      message << _name << " are required.";
      throw std::invalid_argument{message.str()};
    }
  }
  else if (_values.size() != _expectedLength)
  {
    std::stringstream message;
    message << "Expected " << _name << " to be of length " << _expectedLength
      << ", got " << _values.size() << ".";
    throw std::invalid_argument{message.str()};
  }

  if (_output)
    *_output = Eigen::Map<const Eigen::VectorXd>(_values.data(), _values.size());
}

//=============================================================================
Eigen::MatrixXd fitPolynomial(
  double _currTime,
  const Eigen::VectorXd& _currPosition,
  const Eigen::VectorXd& _currVelocity,
  const Eigen::VectorXd& _currAcceleration,
  double _nextTime,
  const Eigen::VectorXd& _nextPosition,
  const Eigen::VectorXd& _nextVelocity,
  const Eigen::VectorXd& _nextAcceleration,
  size_t _numCoefficients)
{
  using aikido::util::SplineProblem;

  assert(_numCoefficients == 2 || _numCoefficients == 4 || _numCoefficients == 6);

  const auto numDofs = _currPosition.size();
  SplineProblem<> splineProblem(
    Eigen::Vector2d(_currTime, _nextTime), _numCoefficients, numDofs);

  assert(_currPosition.size() == numDofs);
  assert(_nextPosition.size() == numDofs);
  splineProblem.addConstantConstraint(0, 0, _currPosition);
  splineProblem.addConstantConstraint(1, 0, _nextPosition);

  if (_numCoefficients >= 4)
  {
    assert(_currVelocity.size() == numDofs);
    assert(_nextVelocity.size() == numDofs);
    splineProblem.addConstantConstraint(0, 1, _currVelocity);
    splineProblem.addConstantConstraint(1, 1, _nextVelocity);
  }

  if (_numCoefficients >= 6)
  {
    assert(_currAcceleration.size() == numDofs);
    assert(_nextAcceleration.size() == numDofs);
    splineProblem.addConstantConstraint(0, 2, _currAcceleration);
    splineProblem.addConstantConstraint(1, 2, _nextAcceleration);
  }

  const auto splineSegment = splineProblem.fit();
  return splineSegment.getCoefficients()[0];
}

//=============================================================================
void extractJointTrajectoryPoint(
  const trajectory_msgs::JointTrajectory& _trajectory,
  size_t _index, size_t _numDofs,
  Eigen::VectorXd* _positions, bool _positionsRequired,
  Eigen::VectorXd* _velocities, bool _velocitiesRequired,
  Eigen::VectorXd* _accelerations, bool _accelerationsRequired)
{
  const auto& waypoint = _trajectory.points[_index];

  try
  {
    checkVector("positions", waypoint.positions, _numDofs,
      _positionsRequired, _positions);
    checkVector("velocities", waypoint.velocities, _numDofs,
      _velocitiesRequired, _velocities);
    checkVector("accelerations", waypoint.accelerations, _numDofs,
      _accelerationsRequired, _accelerations);
  }
  catch (const std::invalid_argument& e)
  {
    std::stringstream message;
    message << "Waypoint " << _index << " is invalid: " << e.what();
    throw std::invalid_argument(message.str());
  }
}

// The rows of vector and matrix will be reordered using permutationMatrix
void reorder(const Eigen::MatrixXd& permutationMatrix,
  const Eigen::VectorXd& inVector,
  const Eigen::MatrixXd& inMatrix,
  Eigen::VectorXd* outVector,
  Eigen::MatrixXd* outMatrix)
{
  *outVector = permutationMatrix*inVector;
  *outMatrix = permutationMatrix*inMatrix;
}

// Convert the mapping to a permutation matrix for matrix operation
void convertToPermutationMatrix(const std::map<size_t, size_t>& map,
  Eigen::MatrixXd* permutationMatrix)
{
  *permutationMatrix = Eigen::MatrixXd::Zero(map.size(), map.size());
  for(auto it = map.begin(); it != map.end(); ++it)
  {
    (*permutationMatrix)(it->second, it->first) = 1.0;
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

  const auto numControlledJoints = space->getNumSubspaces();
  if (jointTrajectory.joint_names.size() != numControlledJoints)
  {
    std::stringstream message;
    message << "Incorrect number of joints: expected "
        << numControlledJoints << ", got "
        << jointTrajectory.joint_names.size() << ".";
    throw std::invalid_argument{message.str()};
  }

  // Check that all joints are single DOF.
  for (size_t i = 0; i < space->getNumSubspaces(); ++i)
  {
    auto n = space->getJointSpace(i)->getJoint()->getNumDofs();
    if (n != 1)
    {
      std::stringstream message;
      message << "Expected 1 dof. Joint " << i << " has " << n << " dofs.";
      throw std::invalid_argument{message.str()};
    }
  }

  if (jointTrajectory.points.size() < 2)
  {
    throw std::invalid_argument{
      "Trajectory must contain two or more waypoints."};
  }

  // Map joint indices between jointTrajectory and space subspaces.
  std::map<size_t, size_t> rosJointToMSSSJoint;

  for (size_t i = 0; i < jointTrajectory.joint_names.size(); ++i)
  {
    std::string joint_name = jointTrajectory.joint_names[i];
    bool found_match = false;

    for (size_t j = 0; j < space->getNumSubspaces(); ++j)
    {
      auto joint = space->getJointSpace(j)->getJoint();
      if (joint_name == joint->getName())
      {
        // Check that the mapping is unique.
        auto search = rosJointToMSSSJoint.find(i);
        if (search != rosJointToMSSSJoint.end())
        {
          std::stringstream message;
          message << "Both subspace[" << search->second << "] and ["
            << j << "] map to joint " << i << " in jointTrajectory.";
          throw std::invalid_argument{message.str()};
        }
        rosJointToMSSSJoint.emplace(std::make_pair(i, j));
        found_match = true;
        break;
      }
    }

    // Check that there is a mapping for every joint.
    if (!found_match)
    {
      std::stringstream message;
      message << joint_name << " does not have a matching joint in space.";
      throw std::invalid_argument{message.str()};
    }
  }

  // Extract the first waypoint to infer the dimensionality of the trajectory.
  Eigen::VectorXd currPosition, currVelocity, currAcceleration;
  extractJointTrajectoryPoint(jointTrajectory, 0, numControlledJoints,
    &currPosition, true, &currVelocity, false, &currAcceleration, false);

  // To be used for reordering to match space's subspace order.
  Eigen::VectorXd currPositionMSSS;
  Eigen::MatrixXd segmentCoefficientsMSSS;

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
    numCoefficients = 2; // linear

  // Convert the ROS trajectory message to an Aikido spline.
  std::unique_ptr<SplineTrajectory> trajectory{new SplineTrajectory{space}};
  auto currState = space->createState();

  Eigen::MatrixXd conversionPermutation;
  convertToPermutationMatrix(rosJointToMSSSJoint, &conversionPermutation);
  const auto& waypoints = jointTrajectory.points;
  for (size_t iwaypoint = 1; iwaypoint < waypoints.size(); ++iwaypoint)
  {
    Eigen::VectorXd nextPosition, nextVelocity, nextAcceleration;
    extractJointTrajectoryPoint(jointTrajectory, iwaypoint, numControlledJoints,
      &nextPosition, isPositionRequired,
      &nextVelocity, isVelocityRequired,
      &nextAcceleration, isAccelerationRequired);

    // Compute spline coefficients for this polynomial segment.
    const auto nextTimeFromStart = waypoints[iwaypoint].time_from_start.toSec();
    const auto segmentDuration = nextTimeFromStart - currTimeFromStart;
    const auto segmentCoefficients = fitPolynomial(
      0., Eigen::VectorXd::Zero(numControlledJoints), currVelocity, currAcceleration,
      segmentDuration, nextPosition - currPosition, nextVelocity, nextAcceleration,
      numCoefficients);

    // Reorder the positions and coefficients using rosJointToMSSSJoint.
    reorder(conversionPermutation, currPosition, segmentCoefficients,
      &currPositionMSSS, &segmentCoefficientsMSSS);

    // Add a segment to the trajectory.
    space->convertPositionsToState(currPositionMSSS, currState);
    trajectory->addSegment(segmentCoefficientsMSSS, segmentDuration, currState);

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

