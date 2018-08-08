#include "aikido/control/ros/RosGripperCommandExecutor.hpp"
#include "aikido/control/ros/util.hpp"
#include <dart/dart.hpp>

namespace aikido {
namespace control {
namespace ros {

RosGripperCommandExecutor::RosGripperCommandExecutor(
    ::ros::NodeHandle node,
    const std::string& serverName,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  : mNode(std::move(node))
  , mCallbackQueue{}
  , mClient{mNode, serverName, &mCallbackQueue}
  , mConnectionTimeout{connectionTimeout}
  , mConnectionPollingPeriod{connectionPollingPeriod}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  // Do nothing.
}

//==============================================================================
RosGripperCommandExecutor::~RosGripperCommandExecutor()
{
  // Do nothing.
}

//==============================================================================
std::future<void> RosGripperCommandExecutor::execute(
    const double goalPosition, const double effort)
{

  // Will check for size matching in the positionsToJointState function
  control_msgs::GripperCommandGoal goal;
  goal.command.position = goalPosition;
  goal.command.max_effort = effort;
  
  bool waitForServer = waitForActionServer<control_msgs::GripperCommandAction,
                                           std::chrono::milliseconds,
                                           std::chrono::milliseconds>(
      mClient, mCallbackQueue, mConnectionTimeout, mConnectionPollingPeriod);

  if (!waitForServer)
    throw std::runtime_error("Unable to connect to action server.");

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
      throw std::runtime_error("Another position command is in progress.");

    mPromise.reset(new std::promise<void>());
    mInProgress = true;
    mGoalHandle = mClient.sendGoal(
        goal,
        boost::bind(&RosGripperCommandExecutor::transitionCallback, this, _1));

    return mPromise->get_future();
  }
}

//==============================================================================
void RosGripperCommandExecutor::transitionCallback(GoalHandle handle)
{
  // This function assumes that mMutex is locked.
std::cout << "goal Position" << "SS" << std::endl;
  using actionlib::TerminalState;
  using Result = control_msgs::GripperCommandAction;

  if (handle.getCommState() == actionlib::CommState::DONE)
  {
    std::stringstream message;
    bool isSuccessful = true;

    // Check the status of the actionlib call. Note that the actionlib call can
    // succeed, even if execution failed.
    const auto terminalState = handle.getTerminalState();
    if (terminalState != TerminalState::SUCCEEDED)
    {
      message << "Action " << terminalState.toString();

      const auto terminalMessage = terminalState.getText();
      if (!terminalMessage.empty())
        message << " (" << terminalMessage << ")";

      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error(message.str())));

      isSuccessful = false;
    }

    // Check the status of execution. This is only possible if the actionlib
    // call succeeded.
    const auto result = handle.getResult();
    if (result && (result->stalled))
    {
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Gripper stalled")));
      isSuccessful = false;
    }

    if (isSuccessful)
      mPromise->set_value();

    mInProgress = false;
  }
}

//==============================================================================
void RosGripperCommandExecutor::step(
    const std::chrono::system_clock::time_point& /*timepoint*/)
{

  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning.
  mCallbackQueue.callAvailable();

  if (!::ros::ok() && mInProgress)
  {
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Detected ROS shutdown.")));
    mInProgress = false;
  }
}

}  // namespace ros
}  // namespace control
}  // ros