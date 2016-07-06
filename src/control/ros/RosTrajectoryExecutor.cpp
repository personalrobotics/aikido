#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/util/StepSequence.hpp>

namespace aikido {
namespace control {
namespace ros {
namespace {

//=============================================================================
std::string getFollowJointTrajectoryErrorMessage(int32_t errorCode)
{
  using Result = control_msgs::FollowJointTrajectoryResult;

  switch (errorCode)
  {
    case Result::SUCCESSFUL:
      return "successful";

    case Result::INVALID_GOAL:
      return "invalid goal";

    case Result::INVALID_JOINTS:
      return "invalid joints";

    case Result::OLD_HEADER_TIMESTAMP:
      return "old header timestamp";

    case Result::PATH_TOLERANCE_VIOLATED:
      return "path tolerance violated";

    case Result::GOAL_TOLERANCE_VIOLATED:
      return "goal tolerance violated";

    default:
      return "unknown";
  }
}

} // namespace

//=============================================================================
RosTrajectoryExecutor::RosTrajectoryExecutor(
      ::dart::dynamics::MetaSkeletonPtr skeleton, 
      ::ros::NodeHandle node,
      const std::string& serverName,
      double timestep,
      double goalTimeTolerance,
      std::chrono::milliseconds connectionTimeout,
      std::chrono::milliseconds connectionPollingRate)
  : mNode{std::move(node)}
  , mCallbackQueue{}
  , mClient{mNode, serverName, &mCallbackQueue}
  , mSkeleton{std::move(skeleton)}
  , mTimestep{timestep}
  , mGoalTimeTolerance{goalTimeTolerance}
  , mConnectionTimeout{connectionTimeout}
  , mConnectionPollingRate{connectionPollingRate}
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");

  if (mTimestep <= 0)
    throw std::invalid_argument("Timestep must be positive.");

  if (mGoalTimeTolerance <= 0)
    throw std::invalid_argument("Goal time tolerance must be positive.");
}

//=============================================================================
RosTrajectoryExecutor::~RosTrajectoryExecutor()
{
  // Do nothing.
  // TODO: Should we wait for the current trajectory to finish executing?
}

//=============================================================================
std::future<void> RosTrajectoryExecutor::execute(
  trajectory::TrajectoryPtr _traj)
{
  using statespace::dart::MetaSkeletonStateSpace;

  if (!_traj)
    throw std::invalid_argument("Trajectory is null.");

  const auto space = std::dynamic_pointer_cast<
    MetaSkeletonStateSpace>(_traj->getStateSpace());
  if (!space)
  {
    throw std::invalid_argument(
      "Trajectory is not in a MetaSkeletonStateSpace.");
  }

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

  // Setup the goal properties.
  // TODO: Also set the goal properties.
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.goal_time_tolerance = ::ros::Duration(mGoalTimeTolerance);

  // Convert the Aikido trajectory into a ROS JointTrajectory.
  util::StepSequence timeSequence{mTimestep, true, 0., _traj->getDuration()};
  const auto numDofs = mSkeleton->getNumDofs();
  const auto numWaypoints = timeSequence.getMaxSteps();
  const auto numDerivatives = std::min<int>(_traj->getNumDerivatives(), 2);

  goal.trajectory.joint_names.reserve(numDofs);
  for (const auto dof : mSkeleton->getDofs())
    goal.trajectory.joint_names.emplace_back(dof->getName());

  goal.trajectory.points.reserve(numWaypoints);
  for (const auto timeFromStart : timeSequence)
  {
    const auto timeAbsolute = _traj->getStartTime() + timeFromStart;
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

  if (!waitForServer())
    throw std::runtime_error("Unable to connect to action server.");

  {
    std::lock_guard<std::mutex> lock(mMutex);

    if (mInProgress)
      throw std::runtime_error("Another trajectory is in progress.");

    mPromise = std::promise<void>();
    mInProgress = true;
    mGoalHandle = mClient.sendGoal(goal, 
      boost::bind(&RosTrajectoryExecutor::transitionCallback, this, _1));

    return mPromise.get_future();
  }
}

//=============================================================================
bool RosTrajectoryExecutor::waitForServer()
{
  using Clock = std::chrono::steady_clock;

  const auto startTime = Clock::now();
  const auto endTime = startTime + mConnectionTimeout;
  auto currentTime = startTime + mConnectionPollingRate;

  while (currentTime < endTime)
  {
    mCallbackQueue.callAvailable();

    // TODO: Is this thread safe?
    if (mClient.isServerConnected())
      return true;

    currentTime += mConnectionPollingRate;
    std::this_thread::sleep_until(currentTime);
  }

  return false;
}

//=============================================================================
void RosTrajectoryExecutor::transitionCallback(GoalHandle _handle)
{
  // This function assumes that mMutex is locked.

  using actionlib::TerminalState;
  using Result = control_msgs::FollowJointTrajectoryResult;

#if 0
  ROS_INFO("Transition: CommState = %s TerminalState = %s",
    _handle.getCommState().toString().c_str(),
    _handle.getTerminalState().toString().c_str());
#endif
  
  if (_handle.getCommState() == actionlib::CommState::DONE)
  {
    std::stringstream message;
    bool isSuccessful = true;

    // Check the status of the actionlib call. Note that the actionlib call can
    // succeed, even if execution failed.
    const auto terminalState = _handle.getTerminalState();
    if (terminalState != TerminalState::SUCCEEDED)
    {
      message << "Action " << terminalState.toString();

      const auto terminalMessage = terminalState.getText();
      if (!terminalMessage.empty())
        message << " (" << terminalMessage << ")";

      isSuccessful = false;
    }
    else
    {
      message << "Execution failed";
    }

    // Check the status of execution. This is only possible if the actionlib
    // call succeeded.
    const auto result = _handle.getResult();
    if (result && result->error_code != Result::SUCCESSFUL)
    {
      message << ": "
        << getFollowJointTrajectoryErrorMessage(result->error_code);

      if (!result->error_string.empty())
        message << " (" << result->error_string << ")";

      isSuccessful = false;
    }

    if (isSuccessful)
    {
      mPromise.set_value();
    }
    else
    {
      mPromise.set_exception(
        std::make_exception_ptr(
          std::runtime_error(message.str())));
    }

    mInProgress = false;
  }
}

//=============================================================================
void RosTrajectoryExecutor::spin()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mCallbackQueue.callAvailable();

  // TODO: Check mGoalTimeTolerance here as a fail-safe.
}

} // namespace ros
} // namespace control
} // namespace aikido
