#include <aikido/control/ros/RosBarrettHandPositionCommandExecutor.hpp>
#include <aikido/control/ros/Conversions.hpp>

namespace aikido {
namespace control {
namespace ros {

using std::chrono::milliseconds;

//=============================================================================
template <typename DurationA, typename DurationB>
RosBarrettHandPositionCommandExecutor::RosBarrettHandPositionCommandExecutor(
      ::ros::NodeHandle node,
      const std::string& serverName,
      const DurationA& connectionTimeout,
      const DurationB& connectionPollingPeriod)
  : mNode(std::move(node))
  , mCallbackQueue{}
  , mClient{mNode, serverName, &mCallbackQueue}
  , mConnectionTimeout{
    std::chrono::duration_cast<milliseconds>(connectionTimeout)}
  , mConnectionPollingPeriod{
    std::chrono::duration_cast<milliseconds>(connectionPollingPeriod)}
  , mInProgress{false}
{
}

//=============================================================================
RosBarrettHandPositionCommandExecutor::~RosBarrettHandPositionCommandExecutor()
{
  // Do nothing.
  // TODO: Should we wait for the current trajectory to finish executing?
}

//=============================================================================
std::future<void> RosBarrettHandPositionCommandExecutor::execute(
  Eigen::VectorXd& goalPositions,
  std::vector<std::string>& jointNames)
{
  static const ::ros::Time invalidTime;
  return execute(goalPositions, jointNames, invalidTime);
}

//=============================================================================
std::future<void> RosBarrettHandPositionCommandExecutor::execute(
  Eigen::VectorXd& goalPositions, 
  std::vector<std::string>& jointNames,
  const ::ros::Time& startTime)
{
  if (goalPositions.size() != 4)
  {
    std::stringstream message;
    message << "GoalPositions must have 4 elements, but ["
      << goalPositions.size() << "] given.";
    throw std::invalid_argument(message.str());
  }

  pr_control_msgs::SetPositionGoal goal;

  // Convert goal positions and joint names to jointstate
  goal.command = toJointState(goalPositions,jointNames);

  goal.command.header.stamp = startTime;

  if (!waitForServer())
    throw std::runtime_error("Unable to connect to action server.");

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
      throw std::runtime_error("Another trajectory is in progress.");

    mPromise = std::promise<void>();
    mInProgress = true;
    mGoalHandle = mClient.sendGoal(goal, 
      boost::bind(&RosBarrettHandPositionCommandExecutor::transitionCallback, this, _1));

    return mPromise.get_future();
  }

}

//=============================================================================
// TODO : This is directly reused from RosTrajectoryExecutor.cpp
// Anyway to reuse this? (assuming we want this here)
bool RosBarrettHandPositionCommandExecutor::waitForServer()
{
  using Clock = std::chrono::steady_clock;

  const auto startTime = Clock::now();
  const auto endTime = startTime + mConnectionTimeout;
  auto currentTime = startTime + mConnectionPollingPeriod;

  while (currentTime < endTime)
  {
    mCallbackQueue.callAvailable();

    // TODO: Is this thread safe?
    if (mClient.isServerConnected())
      return true;

    currentTime += mConnectionPollingPeriod;
    std::this_thread::sleep_until(currentTime);
  }

  return false;
}

//=============================================================================
void RosBarrettHandPositionCommandExecutor::transitionCallback(GoalHandle handle)
{

  using actionlib::TerminalState;
  using Result = pr_control_msgs::SetPositionResult;

  if (handle.getCommState() == actionlib::CommState::DONE)
  {
    std::stringstream message;
    bool isSuccessful = true;

    const auto terminalState = handle.getTerminalState();
    if (terminalState != TerminalState::SUCCEEDED)
    {
      message << "Action " << terminalState.toString();

      const auto terminalMessage = terminalState.getText();
      if (!terminalMessage.empty())
        message << " (" << terminalMessage << ")";

      mPromise.set_exception(std::make_exception_ptr(
          std::runtime_error(message.str())));

      isSuccessful = false;
    }
    else{
      message << "Execution failed.";
    }

    const auto result = handle.getResult();
    if (result && result->success == false)
    {
      if (!result->message.empty())
        message << " (" << result->message << ")";

      mPromise.set_exception(std::make_exception_ptr(
          std::runtime_error(message.str())));

      isSuccessful = false;
    }

    if (isSuccessful)
      mPromise.set_value();

    mInProgress = false;

  }
}

//=============================================================================
// TODO : This is also basically copied from RosTrajectoryExecutor
void RosBarrettHandPositionCommandExecutor::step()
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning.

  mCallbackQueue.callAvailable();

  if (!::ros::ok() && mInProgress)
  {
    mPromise.set_exception(
      std::make_exception_ptr(std::runtime_error("Detected ROS shutdown.")));
    mInProgress = false;
  }
}


} // namespace ros
} // namespace control
} // namespace aikido