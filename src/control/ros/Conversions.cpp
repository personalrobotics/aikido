#include <sstream>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/statespace/dart/SO2Joint.hpp>
#include <aikido/util/Spline.hpp>
#include <dart/dynamics/Joint.hpp>
#include <map>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using SplineTrajectory = aikido::trajectory::Spline;
using aikido::statespace::dart::RnJoint;
using aikido::statespace::dart::SO2Joint;

namespace aikido {
namespace control {
namespace ros {
namespace {

void reorder(std::map<size_t, size_t> indexMap,
  const Eigen::VectorXd& inVector, Eigen::VectorXd* outVector);

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
  Eigen::VectorXd* _accelerations, bool _accelerationsRequired,
  std::map<size_t, size_t> indexMap)
{
  const auto& waypoint = _trajectory.points[_index];

  try
  {
    Eigen::VectorXd trajPos, trajVel, trajAccel;
    checkVector("positions", waypoint.positions, _numDofs,
      _positionsRequired, &trajPos);
    checkVector("velocities", waypoint.velocities, _numDofs,
      _velocitiesRequired, &trajVel);
    checkVector("accelerations", waypoint.accelerations, _numDofs,
      _accelerationsRequired, &trajAccel);

    if ( trajPos.size() > 0 )
      reorder(indexMap, trajPos, _positions);
    if ( trajVel.size() > 0 )
      reorder(indexMap, trajVel, _velocities);
    if ( trajAccel.size() > 0 )
      reorder(indexMap, trajAccel, _accelerations);
  }
  catch (const std::invalid_argument& e)
  {
    std::stringstream message;
    message << "Waypoint " << _index << " is invalid: " << e.what();
    throw std::invalid_argument(message.str());
  }
}

//=============================================================================
// The rows of inVector is reordered in outVector.
void reorder(std::map<size_t, size_t> indexMap,
  const Eigen::VectorXd& inVector, Eigen::VectorXd* outVector)
{
  *outVector = Eigen::VectorXd::Zero(inVector.size());
  for (auto it = indexMap.begin(); it != indexMap.end(); ++it)
  {
    (*outVector)(it->second) = inVector(it->first);
  }
}

//=============================================================================
std::vector<const dart::dynamics::Joint*> findJointByName(
  const dart::dynamics::MetaSkeleton& metaSkeleton,
  const std::string& jointName)
{
  std::vector<const dart::dynamics::Joint*> joints;

  for (size_t i = 0; i < metaSkeleton.getNumJoints(); ++i)
  {
    auto joint = metaSkeleton.getJoint(i);
    if (joint->getName() == jointName)
    {
      joints.emplace_back(metaSkeleton.getJoint(i));
    }
  }

  return joints;
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

  // Check that the names in jointTrajectory are unique.
  auto joint_names = jointTrajectory.joint_names;
  std::sort(joint_names.begin(), joint_names.end());
  auto duplicate_it = std::adjacent_find(std::begin(joint_names), std::end(joint_names));
  if (duplicate_it != std::end(joint_names))
  {
    std::stringstream message;
    message << "JointTrajectory has multiple joints with same name ["
      << *duplicate_it << "].";
    throw std::invalid_argument{message.str()};
  }

  // Check that all joints are single-DOF RnJoint or SO2JOint state spaces.
  for (size_t i = 0; i < space->getNumSubspaces(); ++i)
  {
    auto joint = space->getJointSpace(i)->getJoint();
    auto jointSpace = space->getSubspace(i);
    auto rnJoint = dynamic_cast<RnJoint*>(jointSpace.get());
    auto so2Joint = dynamic_cast<SO2Joint*>(jointSpace.get());

    if (joint->getNumDofs() != 1 || (!rnJoint && !so2Joint))
    {
      std::stringstream message;
      message << "Only single-DOF RnJoint and SO2Joint are supported. Joint "
        << joint->getName() << "(index: " << i << ") is a "
        << joint->getType() << " with "
        << joint->getNumDofs() << " DOFs.";
      throw std::invalid_argument{message.str()};
    }
  }

  if (jointTrajectory.points.size() < 2)
  {
    throw std::invalid_argument{
      "Trajectory must contain two or more waypoints."};
  }

  // Map joint indices between jointTrajectory and space subspaces.
  std::map<size_t, size_t> rosJointToMetaSkeletonJoint;

  auto metaSkeleton = space->getMetaSkeleton();
  auto nJoints = jointTrajectory.joint_names.size();

  for (size_t trajJointIndex = 0; trajJointIndex < nJoints; ++trajJointIndex)
  {
    const auto& jointName = jointTrajectory.joint_names[trajJointIndex];
    auto joints = findJointByName(*metaSkeleton, jointName);

    if (joints.empty())
    {
      std::stringstream message;
      message << "Joint " << jointName << " (trajectory index: "
        << trajJointIndex << ")" << " does not exist in MetaSkeleton.";
      throw std::invalid_argument{message.str()};
    }
    else if  (joints.size() > 1)
    {
      std::stringstream message;
      message << "Multiple (" << joints.size()
        << ") joints have the same name [" << jointName << "] "
        << "in the JointTrajectory.";
      throw std::invalid_argument{message.str()};
    }

    auto joint = joints[0];
    auto metaSkeletonIndex = metaSkeleton->getIndexOf(joint);
    assert(metaSkeletonIndex != dart::dynamics::INVALID_INDEX);

    rosJointToMetaSkeletonJoint.emplace(
      std::make_pair(trajJointIndex, metaSkeletonIndex));
  }

  // Extract the first waypoint to infer the dimensionality of the trajectory.
  Eigen::VectorXd currPosition, currVelocity, currAcceleration;
  extractJointTrajectoryPoint(jointTrajectory, 0, numControlledJoints,
    &currPosition, true, &currVelocity, false, &currAcceleration, false,
    rosJointToMetaSkeletonJoint);

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

  const auto& waypoints = jointTrajectory.points;
  for (size_t iwaypoint = 1; iwaypoint < waypoints.size(); ++iwaypoint)
  {
    Eigen::VectorXd nextPosition, nextVelocity, nextAcceleration;
    extractJointTrajectoryPoint(jointTrajectory, iwaypoint, numControlledJoints,
      &nextPosition, isPositionRequired,
      &nextVelocity, isVelocityRequired,
      &nextAcceleration, isAccelerationRequired,
      rosJointToMetaSkeletonJoint);

    // Compute spline coefficients for this polynomial segment.
    const auto nextTimeFromStart = waypoints[iwaypoint].time_from_start.toSec();
    const auto segmentDuration = nextTimeFromStart - currTimeFromStart;
    const auto segmentCoefficients = fitPolynomial(
      0., Eigen::VectorXd::Zero(numControlledJoints), currVelocity, currAcceleration,
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

