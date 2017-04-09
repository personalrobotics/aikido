#include <sstream>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/statespace/dart/SO2Joint.hpp>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/util/Spline.hpp>
#include <aikido/util/StepSequence.hpp>
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
void extractTrajectoryPoint(
  const std::shared_ptr<MetaSkeletonStateSpace>& space,
  const aikido::trajectory::TrajectoryPtr& trajectory,
  double timeFromStart, const std::map<size_t, size_t>& indexMap,
  trajectory_msgs::JointTrajectoryPoint& waypoint)
{
  const auto numDerivatives = std::min<int>(trajectory->getNumDerivatives(), 1);
  const auto timeAbsolute = trajectory->getStartTime() + timeFromStart;
  
  Eigen::VectorXd tangentVector, jointTangentVector;
  auto state = space->createState();
  trajectory->evaluate(timeAbsolute, state);
  space->logMap(state, tangentVector);
  assert(tangentVector.size() == numDofs);

  // Reorder tangentVectoraccording to the index mapping
  reorder(indexMap, tangentVector, &jointTangentVector);

  waypoint.time_from_start = ::ros::Duration(timeFromStart);
  waypoint.positions.assign(jointTangentVector.data(),
    jointTangentVector.data() + jointTangentVector.size());

  assert(0 <= numDerivatives && numDerivatives <= 2);
  const std::array<std::vector<double>*, 2> derivatives{
    &waypoint.velocities, &waypoint.accelerations};
  for (int iderivative = 1; iderivative <= numDerivatives; ++iderivative)
  {
    trajectory->evaluateDerivative(timeAbsolute, iderivative, tangentVector);
    assert(tangentVector.size() == numDofs);

    // Reorder according to the index mapping
    reorder(indexMap, tangentVector, &jointTangentVector);
    derivatives[iderivative - 1]->assign(jointTangentVector.data(),
      jointTangentVector.data() + jointTangentVector.size());
  }
}

//=============================================================================
// The rows of inVector is reordered in outVector.
void reorder(std::map<size_t, size_t> indexMap,
  const Eigen::VectorXd& inVector, Eigen::VectorXd* outVector)
{
  *outVector = Eigen::VectorXd::Zero(inVector.size());
  for (auto it = indexMap.begin(); it != indexMap.end(); ++it)
    (*outVector)(it->second) = inVector(it->first);
  
}

//=============================================================================
dart::dynamics::Joint* findJointByName(
    dart::dynamics::MetaSkeleton* metaSkeleton, const std::string& jointName)
{
  for (size_t i = 0; i < metaSkeleton->getNumJoints(); ++i)
  {
    if (metaSkeleton->getJoint(i)->getName() == jointName)
      return metaSkeleton->getJoint(i);
  }

  return nullptr;
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

  for (size_t i = 0; i < jointTrajectory.joint_names.size(); ++i)
  {
    std::string jointName = jointTrajectory.joint_names[i];
    auto joint = findJointByName(metaSkeleton.get(), jointName);

    if (!joint)
    {
      std::stringstream message;
      message << "Joint " << jointName << " (index: " << i << ")"
        << " does not exist in metaSkeleton.";
      throw std::invalid_argument{message.str()};
    }
    auto index = metaSkeleton->getIndexOf(joint);
    assert(index != dart::dynamics::INVALID_INDEX);

    rosJointToMetaSkeletonJoint.emplace(std::make_pair(i, index));

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

//=============================================================================
const trajectory_msgs::JointTrajectory convertTrajectoryToRosTrajectory(
  const aikido::trajectory::TrajectoryPtr& trajectory,
  const std::map<std::string, size_t>& indexMap, double timestep)
{
  using statespace::dart::MetaSkeletonStateSpace;
  using statespace::dart::SO2Joint;
  using statespace::dart::RnJoint;

  if (!trajectory)
    throw std::invalid_argument("Trajectory is null.");

  if (timestep <= 0)
    throw std::invalid_argument("Timestep must be positive.");

  const auto space = std::dynamic_pointer_cast<
    MetaSkeletonStateSpace>(trajectory->getStateSpace());
  if (!space)
  {
    throw std::invalid_argument(
      "Trajectory is not in a MetaSkeletonStateSpace.");
  }

  for (size_t i = 0; i < space->getNumSubspaces(); ++i)
  {
    // Supports only RnJoints and SO2Joints.
    auto rnJoint = std::dynamic_pointer_cast<RnJoint>(space->getSubspace(i));
    auto so2Joint = std::dynamic_pointer_cast<SO2Joint>(space->getSubspace(i));
    if (!rnJoint && !so2Joint)
    {
      throw std::invalid_argument(
        "MetaSkeletonStateSpace must contain only RnJonts and SO2Joints.");
    }

    // For RnJoint, supports only 1D.
    if (rnJoint && rnJoint->getDimension() > 1)
    {
      std::stringstream message;
      message << "RnJoint must be 1D. Joint "
        << i << " has " << rnJoint->getDimension() << " dimensions.";
      throw std::invalid_argument{message.str()};
    }
  }

  util::StepSequence timeSequence{timestep, true, 0., trajectory->getDuration()};
  const auto numJoints = space->getNumSubspaces();
  const auto numWaypoints = timeSequence.getMaxSteps();
  const auto metaSkeleton = space->getMetaSkeleton();
  trajectory_msgs::JointTrajectory jointTrajectory;
  jointTrajectory.joint_names.resize(numJoints);

  if (indexMap.size() != numJoints)
  {
    std::stringstream message;
    message << "Trajectory's skeleton has " << numJoints << " joints, but "
      << "indexMap has " << indexMap.size() << " elements.";
    throw std::invalid_argument{message.str()};
  }

  // Assign joint names
  std::map<size_t, size_t> orderMap;

  for (auto it = indexMap.begin(); it != indexMap.end(); ++it)
  {
    auto joint = findJointByName(metaSkeleton.get(), it->first);
    if (!joint)
    {
      std::stringstream message;
      message << "Skeleton does not have joint[" << it->first
        << "], given by indexMap.";
      throw std::invalid_argument{message.str()};
    }


    if (it->second > numJoints) 
    {
      std::stringstream message;
      message << "Skeleton has " << numJoints << " joints, but "
        << "indexMap maps " << it->first << " to joint " << it->second;
      throw std::invalid_argument{message.str()};
    }

    if (jointTrajectory.joint_names[it->second] != "")
    {
      std::stringstream message;
      message << jointTrajectory.joint_names[it->second] << " and " <<
        it->first << " map to the same " << it->second << " joint.";
      throw std::invalid_argument{message.str()};
    }

    jointTrajectory.joint_names[it->second] = it->first;

    auto index = metaSkeleton->getIndexOf(joint);
    assert(index != dart::dynamics::INVALID_INDEX);
    orderMap.emplace(std::make_pair(index, it->second));
  }

  // Evaluate trajectory at each timestep and insert it into jointTrajectory
  jointTrajectory.points.reserve(numWaypoints);
  for (const auto timeFromStart : timeSequence)
  {
    trajectory_msgs::JointTrajectoryPoint waypoint;

    extractTrajectoryPoint(
      space, trajectory, timeFromStart, orderMap, waypoint);

    jointTrajectory.points.emplace_back(waypoint);
  }

  return jointTrajectory;
}

} // namespace ros
} // namespace control
} // namespace aikido

