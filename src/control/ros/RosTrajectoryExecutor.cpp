#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/Rn.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <algorithm>

namespace aikido {
namespace control {
namespace ros {

RosTrajectoryExecutor::RosTrajectoryExecutor( 
      ::dart::dynamics::SkeletonPtr skeleton,
      ::ros::NodeHandle node,
      const std::string& serverName,
      double timestep)
  : mNode(std::move(node))
  , mCallbackQueue()
  , mClient(mNode, serverName, &mCallbackQueue)
  , mSkeleton(std::move(skeleton))
  , mTimestep(timestep)
{
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;

  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");

  if (mTimestep <= 0)
    throw std::invalid_argument("Timestep must be positive.");
}

RosTrajectoryExecutor::~RosTrajectoryExecutor()
{
  // Do nothing.
  // TODO: Should we wait for the current trajectory to finish executing?
}

std::future<void> RosTrajectoryExecutor::execute(
  trajectory::TrajectoryPtr _traj)
{
  using statespace::dart::MetaSkeletonStateSpace;

  if (!_traj)
    throw std::invalid_argument("Traj is null.");

  const auto space = std::dynamic_pointer_cast<
    MetaSkeletonStateSpace>(_traj->getStateSpace());
  if (!space)
  {
    throw std::invalid_argument(
      "Trajectory is not in a MetaSkeletonStateSpace.");
  }

  // Verify that 
  const auto trajectorySkeleton = space->getMetaSkeleton();
  for (const auto dof : mSkeleton->getDofs())
  {
    const auto idof = trajectorySkeleton->getIndexOf(dof, false);
    if (idof == dart::dynamics::INVALID_INDEX)
    {
      std::stringstream msg;
      msg << "Trajectory is missing DegreeOfFreedom '" << dof->getName()
          << "' for Skeleton '" << dof->getSkeleton()->getName() << "'.";
      throw std::invalid_argument(msg.str());
    }
  }

  // Discretize time at the desired timestep.
  const auto numDofs = mSkeleton->getNumDofs();
  const auto numDerivatives = std::min<int>(_traj->getNumDerivatives(), 2);
  const auto numWaypoints = static_cast<int>(
    _traj->getDuration() / mTimestep + 1);
  std::vector<double> waypointTimes;
  waypointTimes.reserve(numWaypoints + 1);

  for (int iwaypoint = 0; iwaypoint < numWaypoints; ++iwaypoint)
    waypointTimes.emplace_back(iwaypoint * mTimestep);

  waypointTimes.emplace_back(_traj->getEndTime());

  // Convert the Aikido trajectory into a ROS JointTrajectory.
  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.reserve(numDofs);
  for (const auto dof : mSkeleton->getDofs())
    goal.trajectory.joint_names.emplace_back(dof->getName());

  goal.trajectory.points.reserve(waypointTimes.size());
  for (const auto timeFromStart : waypointTimes)
  {
    const auto timeAbsolute = _traj->getStartTime();
    trajectory_msgs::JointTrajectoryPoint waypoint;

    Eigen::VectorXd tangentVector;
    auto state = space->createState();
    _traj->evaluate(timeAbsolute, state);
    space->logMap(state, tangentVector);
    assert(tangentVector.size() == numDofs);

    waypoint.time_from_start = ::ros::Duration(timeFromStart);
    waypoint.positions.assign(tangentVector.data(),
      tangentVector.data() + tangentVector.size());

    assert(0 <= numDerivatives && numDerivatives <= 2);
    const std::array<std::vector<double>*, 2> derivatives{
      &waypoint.velocities, &waypoint.accelerations};
    for (int iderivative = 1; iderivative <= numDerivatives; ++iderivative)
    {
      _traj->evaluateDerivative(timeAbsolute, iderivative, tangentVector);
      assert(tangentVector.size() == numDofs);

      derivatives[iderivative - 1]->assign(tangentVector.data(),
        tangentVector.data() + tangentVector.size());
    }

    goal.trajectory.points.emplace_back(waypoint);
  }

  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress)
    throw std::runtime_error("Another trajectory is in progress.");

  // TODO: Wait for actionlib to start up.
  if (!mClient.isServerConnected())
    throw std::runtime_error("Not connected to the ActionServer.");

  mPromise = std::promise<void>();
  mInProgress = true;
  mGoalHandle = mClient.sendGoal(goal, 
    boost::bind(&RosTrajectoryExecutor::transitionCallback, this, _1));

  return mPromise.get_future();
}

void RosTrajectoryExecutor::transitionCallback(GoalHandle _handle)
{
  if (_handle.getCommState() == actionlib::CommState::DONE)
  {
    const auto terminalState = _handle.getTerminalState();
    if (terminalState == actionlib::TerminalState::SUCCEEDED)
    {
      mPromise.set_value();
    }
    else
    {
      std::stringstream message;
      message << "Trajectory execution " << terminalState.toString() << ": "
        << terminalState.getText();

      mPromise.set_exception(
        std::make_exception_ptr(
          std::runtime_error(message.str())));
    }
  }
}

void RosTrajectoryExecutor::spin()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mCallbackQueue.callAvailable();
}

} // namespace ros
} // namespace control
} // namespace aikido
