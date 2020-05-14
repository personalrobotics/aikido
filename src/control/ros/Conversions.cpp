#include "aikido/control/ros/Conversions.hpp"

#include <sstream>
#include <unordered_set>

#include <dart/dynamics/Joint.hpp>

#include "aikido/common/Spline.hpp"
#include "aikido/common/StepSequence.hpp"
#include "aikido/control/ros/Conversions.hpp"
#include "aikido/statespace/dart/RnJoint.hpp"
#include "aikido/statespace/dart/SO2Joint.hpp"

using aikido::statespace::CartesianProduct;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using SplineTrajectory = aikido::trajectory::Spline;
using aikido::statespace::dart::R1Joint;
using aikido::statespace::dart::SO2Joint;

namespace aikido {
namespace control {
namespace ros {
namespace {

/// Reorder the elements of a vector according to an index map.
/// \param[in] indexMap Map denoting the reordering between in and out indices.
/// \param[in] inVector Vector to reorder.
/// \param[out] outVector Reordered vector.
void reorder(
    const std::vector<std::pair<std::size_t, std::size_t>>& indexMap,
    const Eigen::VectorXd& inVector,
    Eigen::VectorXd& outVector);

/// Checks if a vector is valid.
/// \param[in] name Name. Provides context to what the vector is representing.
/// \param[in] values The std::vector to check.
/// \param[in] expectedLength Expected size of the vector.
/// \param[in] isRequired True if the vector is expected to contain entries.
/// \param[out] output Converted Eigen vector.
void checkVector(
    const std::string& name,
    const std::vector<double>& values,
    std::size_t expectedLength,
    bool isRequired,
    Eigen::VectorXd& output);

/// Fits a polynomial between two states [t, position, velocity, acceleration].
/// \param[in] currTime Start time.
/// \param[in] currPosition Start position.
/// \param[in] currVelocity Start Velocity.
/// \param[in] currAcceleration Start Acceleration.
/// \param[in] nextTime End time.
/// \param[in] nextPosition End Position.
/// \param[in] nextVelocity End Velocity.
/// \param[in] nextAcceleration End Acceleration.
/// \param[in] numCoefficients Degree of the polynomial to fit.
Eigen::MatrixXd fitPolynomial(
    double currTime,
    const Eigen::VectorXd& currPosition,
    const Eigen::VectorXd& currVelocity,
    const Eigen::VectorXd& currAcceleration,
    double nextTime,
    const Eigen::VectorXd& nextPosition,
    const Eigen::VectorXd& nextVelocity,
    const Eigen::VectorXd& nextAcceleration,
    std::size_t numCoefficients);

/// Extract a state on the joint trajectory given an index.
/// \param[in] trajectory Joint trajectory to extract points from.
/// \param[in] index Index of the point on the trajectory to extract.
/// \param[out] positions Extracted positions.
/// \param[in] positionsRequired True if positions should be extracted.
/// \param[out] velocities Extracted velocities corresponding to \c positions.
/// \param[in] velocitiesRequired True if velocities should be extracted.
/// \param[out] accelerations Extracted accelerations corresponding to \c
/// positions.
/// \param[in] accelerationsRequired True if accelerations should be extracted.
/// \param[in] indexMap Map denoting the correct ordering of trajectory data.
/// This is required in case the trajectory's joint indexing is different than
/// metaskeleton joint indexing.
/// \param[in] unspecifiedJoints Joints whose data is not required. Assumed to
/// be static at the current position.
/// \param[in] startPositions Start positions of the joints.
void extractJointTrajectoryPoint(
    const trajectory_msgs::JointTrajectory& _trajectory,
    std::size_t index,
    std::size_t numDofs,
    Eigen::VectorXd& positions,
    bool positionsRequired,
    Eigen::VectorXd& velocities,
    bool velocitiesRequired,
    Eigen::VectorXd& accelerations,
    bool accelerationsRequired,
    const std::vector<std::pair<std::size_t, std::size_t>>& indexMap,
    const std::vector<std::size_t>& unspecifiedJoints,
    const Eigen::VectorXd& startPositions);

/// Extract a state on the trajectory given a timepoint.
/// \param[in] space MetaSkeletonStateSpace of the trajectory.
/// \param[in] trajectory Trajectory to extract point from.
/// \param[in] timeFromStart Timepoint to extract trajectory point at.
/// \param[out] waypoint The extracted trajectory point.
void extractTrajectoryPoint(
    const std::shared_ptr<const MetaSkeletonStateSpace>& space,
    const aikido::trajectory::ConstTrajectoryPtr& trajectory,
    double timeFromStart,
    trajectory_msgs::JointTrajectoryPoint& waypoint);

//==============================================================================
void reorder(
    const std::vector<std::pair<std::size_t, std::size_t>>& indexMap,
    const Eigen::VectorXd& inVector,
    Eigen::VectorXd& outVector)
{
  assert(indexMap.size() == static_cast<std::size_t>(inVector.size()));
  outVector.resize(inVector.size());
  for (const auto& index : indexMap)
    outVector[index.second] = inVector[index.first];
}

//==============================================================================
void checkVector(
    const std::string& _name,
    const std::vector<double>& _values,
    std::size_t _expectedLength,
    bool _isRequired,
    Eigen::VectorXd& _output)
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

  _output = Eigen::Map<const Eigen::VectorXd>(_values.data(), _values.size());
}

//==============================================================================
Eigen::MatrixXd fitPolynomial(
    double _currTime,
    const Eigen::VectorXd& _currPosition,
    const Eigen::VectorXd& _currVelocity,
    const Eigen::VectorXd& _currAcceleration,
    double _nextTime,
    const Eigen::VectorXd& _nextPosition,
    const Eigen::VectorXd& _nextVelocity,
    const Eigen::VectorXd& _nextAcceleration,
    std::size_t _numCoefficients)
{
  using aikido::common::SplineProblem;

  assert(
      _numCoefficients == 2 || _numCoefficients == 4 || _numCoefficients == 6);

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

//==============================================================================
void extractJointTrajectoryPoint(
    const trajectory_msgs::JointTrajectory& _trajectory,
    std::size_t _index,
    std::size_t _numDofs,
    Eigen::VectorXd& _positions,
    bool _positionsRequired,
    Eigen::VectorXd& _velocities,
    bool _velocitiesRequired,
    Eigen::VectorXd& _accelerations,
    bool _accelerationsRequired,
    const std::vector<std::pair<std::size_t, std::size_t>>& indexMap,
    const std::vector<std::size_t>&
        unspecifiedJoints, // joints to get from startPositions
    const Eigen::VectorXd& startPositions)
{
  const auto& waypoint = _trajectory.points[_index];

  // TODO: are these / should these be copies?
  auto positions = waypoint.positions;
  auto velocities = waypoint.velocities;
  auto accelerations = waypoint.accelerations;

  positions.reserve(positions.size() + unspecifiedJoints.size());
  for (auto unspecifiedJoint : unspecifiedJoints)
    positions.emplace_back(startPositions[unspecifiedJoint]);

  if (velocities.size() > 0)
    velocities.resize(velocities.size() + unspecifiedJoints.size(), 0.0);

  if (accelerations.size() > 0)
    accelerations.resize(accelerations.size() + unspecifiedJoints.size(), 0.0);

  try
  {
    Eigen::VectorXd trajPos, trajVel, trajAccel;
    checkVector("positions", positions, _numDofs, _positionsRequired, trajPos);
    checkVector(
        "velocities", velocities, _numDofs, _velocitiesRequired, trajVel);
    checkVector(
        "accelerations",
        accelerations,
        _numDofs,
        _accelerationsRequired,
        trajAccel);

    if (trajPos.size() > 0)
      reorder(indexMap, trajPos, _positions);
    if (trajVel.size() > 0)
      reorder(indexMap, trajVel, _velocities);
    if (trajAccel.size() > 0)
      reorder(indexMap, trajAccel, _accelerations);
  }
  catch (const std::invalid_argument& e)
  {
    std::stringstream message;
    message << "Waypoint " << _index << " is invalid: " << e.what();
    throw std::invalid_argument(message.str());
  }
}

//==============================================================================
void extractTrajectoryPoint(
    const std::shared_ptr<const MetaSkeletonStateSpace>& space,
    const aikido::trajectory::ConstTrajectoryPtr& trajectory,
    double timeFromStart,
    trajectory_msgs::JointTrajectoryPoint& waypoint)
{
  const auto numDerivatives = std::min<int>(trajectory->getNumDerivatives(), 1);
  const auto timeAbsolute = trajectory->getStartTime() + timeFromStart;
  const int numDof = space->getDimension();
  DART_UNUSED(numDof);

  Eigen::VectorXd tangentVector;
  auto state = space->createState();
  trajectory->evaluate(timeAbsolute, state);
  space->logMap(state, tangentVector);

  assert(tangentVector.size() == numDof);

  waypoint.time_from_start = ::ros::Duration(timeFromStart);
  waypoint.positions.assign(
      tangentVector.data(), tangentVector.data() + tangentVector.size());

  assert(0 <= numDerivatives && numDerivatives <= 2);
  const std::array<std::vector<double>*, 2> derivatives{
      &waypoint.velocities, &waypoint.accelerations};
  for (int iDerivative = 1; iDerivative <= numDerivatives; ++iDerivative)
  {
    trajectory->evaluateDerivative(timeAbsolute, iDerivative, tangentVector);
    assert(tangentVector.size() == numDof);

    derivatives[iDerivative - 1]->assign(
        tangentVector.data(), tangentVector.data() + tangentVector.size());
  }
}

} // namespace

//==============================================================================
std::unique_ptr<SplineTrajectory> toSplineJointTrajectory(
    const std::shared_ptr<MetaSkeletonStateSpace>& space,
    const trajectory_msgs::JointTrajectory& jointTrajectory)
{
  return toSplineJointTrajectory(space, jointTrajectory, Eigen::VectorXd());
}

//==============================================================================
std::unique_ptr<SplineTrajectory> toSplineJointTrajectory(
    const std::shared_ptr<MetaSkeletonStateSpace>& space,
    const trajectory_msgs::JointTrajectory& jointTrajectory,
    const Eigen::VectorXd& startPositions)
{
  if (!space)
    throw std::invalid_argument{"StateSpace must be non-null."};

  bool paddingMode = false;

  // Check that the number of joints specified in JointTrajectory or start
  // position match the state space.
  // 1. A trajectory must either specify as many joints as are in the space (no
  //    padding) or fewer joints than are in the space (padding).
  // 2. A start position must either have a size of 0 (no padding) or a size
  //    matching the number of joints in the space (padding).
  const auto numControlledJoints = space->getNumSubspaces();
  const auto numTrajectoryJoints = jointTrajectory.joint_names.size();
  if (numTrajectoryJoints > numControlledJoints)
  {
    std::stringstream message;
    message << "Incorrect number of joints: expected " << numControlledJoints
            << " or a non-negative number less than that, got "
            << numTrajectoryJoints << ".";
    throw std::invalid_argument{message.str()};
  }
  else if (numTrajectoryJoints < numControlledJoints)
  {
    paddingMode = true;

    if (static_cast<std::size_t>(startPositions.size()) != numControlledJoints)
    {
      std::stringstream message;
      message << "Incorrect number of joints in configuration: expected "
              << numControlledJoints << ", got " << startPositions.size()
              << ".";
      throw std::invalid_argument{message.str()};
    }
  }
  // When numTrajectoryJoints == numControlledJoints, don't care the size of
  // startPositions

  // Check that the names in jointTrajectory are unique.
  std::vector<std::string> joint_names(
      jointTrajectory.joint_names.begin(), jointTrajectory.joint_names.end());
  std::sort(joint_names.begin(), joint_names.end());
  auto duplicate = std::adjacent_find(joint_names.begin(), joint_names.end());
  if (duplicate != joint_names.end())
  {
    std::stringstream message;
    message << "JointTrajectory has multiple joints with same name ["
            << *duplicate << "].";
    throw std::invalid_argument{message.str()};
  }

  // Check that all joints are R1Joint or SO2Joint state spaces.
  // Add subspaces for spline trajectory creation.
  std::vector<aikido::statespace::ConstStateSpacePtr> subspaces;
  for (std::size_t i = 0; i < numControlledJoints; ++i)
  {
    auto jointSpace = space->getJointSpace(i);
    auto properties = jointSpace->getProperties();
    auto r1Joint = std::dynamic_pointer_cast<const R1Joint>(jointSpace);
    auto so2Joint = std::dynamic_pointer_cast<const SO2Joint>(jointSpace);

    if (properties.getNumDofs() != 1 || (!r1Joint && !so2Joint))
    {
      std::stringstream message;
      message << "Only R1Joint and SO2Joint are supported. Joint "
              << properties.getName() << "(index: " << i << ") is a "
              << properties.getType() << " with " << properties.getNumDofs()
              << " DOFs.";
      throw std::invalid_argument{message.str()};
    }
    else if (!r1Joint)
    {
      subspaces.emplace_back(std::make_shared<aikido::statespace::SO2>());
    }
    else
    {
      subspaces.emplace_back(std::make_shared<aikido::statespace::R1>());
    }
  }

  if (jointTrajectory.points.size() < 2)
  {
    throw std::invalid_argument{
        "Trajectory must contain two or more waypoints."};
  }

  // Map joint indices between jointTrajectory and space subspaces.
  std::vector<std::pair<std::size_t, std::size_t>> rosJointToMetaSkeletonJoint;
  std::unordered_set<std::size_t> specifiedMetaSkeletonJoints;

  const auto metaSkeletonProperties = space->getProperties();
  for (std::size_t trajIndex = 0; trajIndex < numTrajectoryJoints; ++trajIndex)
  {
    const auto& dofName = jointTrajectory.joint_names[trajIndex];
    auto metaSkeletonIndex = metaSkeletonProperties.getDofIndex(dofName);

    rosJointToMetaSkeletonJoint.emplace_back(
        std::make_pair(trajIndex, metaSkeletonIndex));
    specifiedMetaSkeletonJoints.insert(metaSkeletonIndex);
  }

  // Add unspecified joint mappings to rosJointToMetaSkeletonJoint

  std::vector<std::size_t> unspecifiedMetaSkeletonJoints;
  unspecifiedMetaSkeletonJoints.reserve(
      numControlledJoints - numTrajectoryJoints);
  if (paddingMode)
  {
    for (std::size_t metaSkeletonIndex = 0;
         metaSkeletonIndex < numControlledJoints;
         ++metaSkeletonIndex)
    {
      if (specifiedMetaSkeletonJoints.find(metaSkeletonIndex)
          == specifiedMetaSkeletonJoints.end())
      {
        // Unspecified joints will be added to the end of the vector
        unspecifiedMetaSkeletonJoints.emplace_back(metaSkeletonIndex);
        rosJointToMetaSkeletonJoint.emplace_back(std::make_pair(
            rosJointToMetaSkeletonJoint.size(), metaSkeletonIndex));
      }
    }
  }

  // Extract the first waypoint to infer the dimensionality of the trajectory.
  Eigen::VectorXd currPosition, currVelocity, currAcceleration;
  extractJointTrajectoryPoint(
      jointTrajectory,
      0,
      numControlledJoints,
      currPosition,
      true,
      currVelocity,
      false,
      currAcceleration,
      false,
      rosJointToMetaSkeletonJoint,
      unspecifiedMetaSkeletonJoints,
      startPositions);

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
  // Add a segment to the trajectory.
  auto compoundSpace
      = std::make_shared<const aikido::statespace::CartesianProduct>(subspaces);
  std::unique_ptr<SplineTrajectory> trajectory{
      new SplineTrajectory{compoundSpace}};
  auto currState = compoundSpace->createState();

  // Temporary States
  auto nextState = compoundSpace->createState();
  auto invCurrState = compoundSpace->createState();
  auto diffState = compoundSpace->createState();

  const auto& waypoints = jointTrajectory.points;
  for (std::size_t iWaypoint = 1; iWaypoint < waypoints.size(); ++iWaypoint)
  {
    Eigen::VectorXd nextPosition;
    Eigen::VectorXd nextVelocity;
    Eigen::VectorXd nextAcceleration;
    extractJointTrajectoryPoint(
        jointTrajectory,
        iWaypoint,
        numControlledJoints,
        nextPosition,
        isPositionRequired,
        nextVelocity,
        isVelocityRequired,
        nextAcceleration,
        isAccelerationRequired,
        rosJointToMetaSkeletonJoint,
        unspecifiedMetaSkeletonJoints,
        startPositions);

    // Calculate diffVec for Spline conversion
    Eigen::VectorXd diffPosition;
    compoundSpace->expMap(currPosition, currState);
    compoundSpace->expMap(nextPosition, nextState);

    compoundSpace->getInverse(currState, invCurrState);
    compoundSpace->compose(nextState, invCurrState, diffState);
    compoundSpace->logMap(diffState, diffPosition);

    // Compute spline coefficients for this polynomial segment.
    const auto nextTimeFromStart = waypoints[iWaypoint].time_from_start.toSec();
    const auto segmentDuration = nextTimeFromStart - currTimeFromStart;
    const auto segmentCoefficients = fitPolynomial(
        0.,
        Eigen::VectorXd::Zero(numControlledJoints),
        currVelocity,
        currAcceleration,
        segmentDuration,
        diffPosition,
        nextVelocity,
        nextAcceleration,
        numCoefficients);

    trajectory->addSegment(segmentCoefficients, segmentDuration, currState);

    // Advance to the next segment.
    currPosition = nextPosition;
    currVelocity = nextVelocity;
    currAcceleration = nextAcceleration;
    currTimeFromStart = nextTimeFromStart;
  }

  return trajectory;
}

//==============================================================================
trajectory_msgs::JointTrajectory toRosJointTrajectory(
    const aikido::trajectory::ConstTrajectoryPtr& trajectory, double timestep)
{
  using statespace::dart::MetaSkeletonStateSpace;
  using statespace::dart::R1Joint;
  using statespace::dart::SO2Joint;

  if (!trajectory)
    throw std::invalid_argument("Trajectory is null.");

  if (timestep <= 0)
    throw std::invalid_argument("Timestep must be positive.");

  const auto space = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      trajectory->getStateSpace());
  if (!space)
  {
    throw std::invalid_argument(
        "Trajectory is not in a MetaSkeletonStateSpace.");
  }

  for (std::size_t i = 0; i < space->getNumSubspaces(); ++i)
  {
    auto jointSpace = space->getJointSpace(i);

    // Supports only R1Joints and SO2Joints.
    auto r1Joint = std::dynamic_pointer_cast<const R1Joint>(jointSpace);
    auto so2Joint = std::dynamic_pointer_cast<const SO2Joint>(jointSpace);
    if (!r1Joint && !so2Joint)
    {
      throw std::invalid_argument(
          "MetaSkeletonStateSpace must contain only R1Joints and SO2Joints.");
    }

    // For RnJoint, supports only 1D.
    if (r1Joint && r1Joint->getDimension() != 1)
    {
      std::stringstream message;
      message << "R1Joint must be 1D. Joint " << i << " has "
              << r1Joint->getDimension() << " dimensions.";
      throw std::invalid_argument{message.str()};
    }
  }

  common::StepSequence timeSequence{
      timestep, true, true, 0., trajectory->getDuration()};
  const auto numJoints = space->getNumSubspaces();
  const auto numWaypoints = timeSequence.getLength();
  trajectory_msgs::JointTrajectory jointTrajectory;
  jointTrajectory.joint_names.reserve(numJoints);

  for (std::size_t i = 0; i < numJoints; ++i)
  {
    const auto jointProperties = space->getJointSpace(i)->getProperties();
    const auto jointDofNames = jointProperties.getDofNames();

    if (jointDofNames.size() != 1)
    {
      std::stringstream message;
      message << "Joint " << jointProperties.getName() << " of type "
              << jointProperties.getType() << " has " << jointDofNames.size()
              << " DOFs.";
      throw std::invalid_argument{message.str()};
    }

    const auto jointDofName = jointDofNames[0];
    jointTrajectory.joint_names.emplace_back(jointDofName);
  }

  // Evaluate trajectory at each timestep and insert it into jointTrajectory
  jointTrajectory.points.reserve(numWaypoints);
  for (const auto& timeFromStart : timeSequence)
  {
    trajectory_msgs::JointTrajectoryPoint waypoint;

    extractTrajectoryPoint(space, trajectory, timeFromStart, waypoint);

    jointTrajectory.points.emplace_back(waypoint);
  }

  return jointTrajectory;
}

//==============================================================================
sensor_msgs::JointState positionsToJointState(
    const Eigen::VectorXd& goalPositions,
    const std::vector<std::string>& jointNames)
{
  if (static_cast<std::size_t>(goalPositions.size()) != jointNames.size())
  {
    std::stringstream message;
    message << "The size of goalPositions (" << goalPositions.size()
            << ") must be the same as jointNames (" << jointNames.size()
            << ")!";
    throw std::invalid_argument(message.str());
  }

  sensor_msgs::JointState jointState;

  jointState.name = std::move(jointNames);
  jointState.position.assign(
      goalPositions.data(), goalPositions.data() + goalPositions.size());

  return jointState;
}

} // namespace ros
} // namespace control
} // namespace aikido
