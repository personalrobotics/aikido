#include <aikido/control/ros/Conversions.hpp>
#include <aikido/control/ros/RosPositionCommandExecutor.hpp>
#include <aikido/control/ros/util.hpp>

namespace aikido {
namespace control {
namespace ros {

using std::chrono::milliseconds;

//==============================================================================
RosPositionCommandExecutor::RosPositionCommandExecutor(
    ::ros::NodeHandle node,
    const std::string& serverName,
    std::vector<std::string> jointNames,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  : mNode(std::move(node))
  , mCallbackQueue{}
  , mClient{mNode, serverName, &mCallbackQueue}
  , mConnectionTimeout{connectionTimeout}
  , mConnectionPollingPeriod{connectionPollingPeriod}
  , mInProgress{false}
  , mJointNames(std::move(jointNames))
{
  // Do nothing.
}

//==============================================================================
RosPositionCommandExecutor::~RosPositionCommandExecutor()
{
  // Do nothing.
}

//==============================================================================
std::future<void> RosPositionCommandExecutor::execute(
    const Eigen::VectorXd& goalPositions)
{
  pr_control_msgs::SetPositionGoal goal;

  // Convert goal positions and joint names to jointstate
  // Will check for size matching in the positionsToJointState function
  goal.command = positionsToJointState(goalPositions, mJointNames);

  bool waitForServer = waitForActionServer<pr_control_msgs::SetPositionAction,
                                           std::chrono::milliseconds,
                                           std::chrono::milliseconds>(
      mClient, mCallbackQueue, mConnectionTimeout, mConnectionPollingPeriod);

  if (!waitForServer)
    throw std::runtime_error("Unable to connect to action server.");

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
      throw std::runtime_error("Another trajectory is in progress.");

    mPromise = std::promise<void>();
    mInProgress = true;
    mGoalHandle = mClient.sendGoal(
        goal,
        boost::bind(&RosPositionCommandExecutor::transitionCallback, this, _1));

    return mPromise.get_future();
  }
}

//==============================================================================
void RosPositionCommandExecutor::transitionCallback(GoalHandle handle)
{
  if (handle.getCommState() == actionlib::CommState::DONE)
  {
    std::stringstream message;
    bool isSuccessful = true;

    const auto terminalState = handle.getTerminalState();
    if (terminalState != actionlib::TerminalState::SUCCEEDED)
    {
      message << "Action " << terminalState.toString();

      const auto terminalMessage = terminalState.getText();
      if (!terminalMessage.empty())
        message << " (" << terminalMessage << ")";

      mPromise.set_exception(
          std::make_exception_ptr(std::runtime_error(message.str())));

      isSuccessful = false;
    }
    else
    {
      message << "Execution failed.";
    }

    const auto result = handle.getResult();
    if (result && result->success == false)
    {
      if (!result->message.empty())
        message << " (" << result->message << ")";

      mPromise.set_exception(
          std::make_exception_ptr(std::runtime_error(message.str())));

      isSuccessful = false;
    }

    if (isSuccessful)
      mPromise.set_value();

    mInProgress = false;
  }
}

//==============================================================================
void RosPositionCommandExecutor::step()
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
