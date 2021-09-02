#include "aikido/control/ros/RosJointGroupCommandClient.hpp"

#include "aikido/control/ros/Conversions.hpp"
#include "aikido/control/ros/util.hpp"

namespace aikido {
namespace control {
namespace ros {

using std::chrono::milliseconds;

//==============================================================================
RosJointGroupCommandClient::RosJointGroupCommandClient(
    ::ros::NodeHandle node,
    const std::string& serverName,
    std::vector<std::string> jointNames,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  : mNode(std::move(node))
  , mCallbackQueue{}
  , mClient{mNode, serverName, &mCallbackQueue}
  , mJointNames(std::move(jointNames))
  , mConnectionTimeout{connectionTimeout}
  , mConnectionPollingPeriod{connectionPollingPeriod}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  // Do nothing.
}

//==============================================================================
RosJointGroupCommandClient::~RosJointGroupCommandClient()
{
  // Do nothing.
}

//==============================================================================
std::future<int> RosJointGroupCommandClient::execute(
    ExecutorType type, const std::vector<double> goal, ::ros::Duration timeout)
{
  auto promise = new std::promise<int>();

  // Convert goal positions and joint names to jointstate
  // Will check for size matching in the positionsToJointState function
  pr_control_msgs::JointGroupCommandGoal commandGoal;
  commandGoal.joint_names = mJointNames;
  commandGoal.command.time_from_start = timeout;
  switch (type)
  {
    case ExecutorType::kPOSITION:
      commandGoal.command.positions = goal;
      break;
    case ExecutorType::kVELOCITY:
      commandGoal.command.velocities = goal;
      break;
    case ExecutorType::kEFFORT:
      commandGoal.command.effort = goal;
      break;
    default:
      promise->set_exception(std::make_exception_ptr(
          std::runtime_error("Unsupported command type.")));
      return promise->get_future();
  }

  bool waitForServer = waitForActionServer<
      pr_control_msgs::JointGroupCommandAction,
      std::chrono::milliseconds,
      std::chrono::milliseconds>(
      mClient, mCallbackQueue, mConnectionTimeout, mConnectionPollingPeriod);

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (!waitForServer)
    {
      promise->set_exception(std::make_exception_ptr(std::runtime_error(
          "Unable to connect to action server (is the controller running?)")));
      mInProgress = false;
      return promise->get_future();
    }

    // Note: no need to cancel previous goal
    // Will be preempted by controller
    if (mInProgress)
    {
      mPromise->set_exception(std::make_exception_ptr(
          std::runtime_error("Action PREEMPTED by new command.")));
    }

    mPromise.reset(promise);
    mInProgress = true;

    mGoalHandle = mClient.sendGoal(
        commandGoal,
        boost::bind(&RosJointGroupCommandClient::transitionCallback, this, _1));

    return mPromise->get_future();
  }
}

//==============================================================================
void RosJointGroupCommandClient::transitionCallback(GoalHandle handle)
{
  int error_code = 0;

  if (handle != mGoalHandle)
  {
    // Expired goal
    return;
  }

  if (handle.getCommState() == actionlib::CommState::DONE)
  {
    std::stringstream message;
    bool isSuccessful = true;

    const auto terminalState = handle.getTerminalState();
    message << "Action " << terminalState.toString() << "; ";

    const auto result = handle.getResult();
    if (result)
    {
      // Communicate error code
      error_code = result->error_code;
      if (result->error_code)
      {
        message << result->error_string;
        ROS_WARN_STREAM_NAMED("RosJointGroupCommandClient", message.str());
      }
    }
    else
    {
      // Only communication is exception
      isSuccessful = false;
    }

    if (isSuccessful)
    {
      mPromise->set_value(error_code);
    }
    else
    {
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error(message.str())));
    }

    mInProgress = false;
  }
}

//==============================================================================
void RosJointGroupCommandClient::step()
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

//==============================================================================
void RosJointGroupCommandClient::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning.

  mClient.cancelAllGoals();
}

} // namespace ros
} // namespace control
} // namespace aikido
