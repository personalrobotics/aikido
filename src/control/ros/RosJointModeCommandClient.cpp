#include "aikido/control/ros/RosJointModeCommandClient.hpp"

#include "aikido/control/ros/Conversions.hpp"
#include "aikido/control/ros/util.hpp"

namespace aikido {
namespace control {
namespace ros {

using std::chrono::milliseconds;

inline int intFromMode(hardware_interface::JointCommandModes mode) {
  switch(mode) {
    case hardware_interface::JointCommandModes::BEGIN:
      return -1;
    case hardware_interface::JointCommandModes::MODE_POSITION:
      return 0;
    case hardware_interface::JointCommandModes::MODE_VELOCITY:
      return 1;
    case hardware_interface::JointCommandModes::MODE_EFFORT:
      return 2;
    case hardware_interface::JointCommandModes::NOMODE:
      return 3;
    case hardware_interface::JointCommandModes::EMERGENCY_STOP:
      return 4;
    case hardware_interface::JointCommandModes::SWITCHING:
      return 5;
    default:
      ROS_WARN_STREAM("Setting error mode to'" <<6<< "'.");
      return 6;
  }
}

//==============================================================================
RosJointModeCommandClient::RosJointModeCommandClient(
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
RosJointModeCommandClient::~RosJointModeCommandClient()
{
  // Do nothing.
}

//==============================================================================
std::future<int> RosJointModeCommandClient::execute(const std::vector<hardware_interface::JointCommandModes>& goal)
{
  auto promise = new std::promise<int>();

  // Convert goal positions and joint names to jointstate
  // Will check for size matching in the positionsToJointState function
  pr_control_msgs::JointModeCommandGoal commandGoal;
  commandGoal.joint_names = mJointNames;

  std::vector<int> goal_modes;
  for(auto mode: goal)
    goal_modes.push_back(intFromMode(mode));
  commandGoal.modes = goal_modes;

  bool waitForServer = waitForActionServer<
      pr_control_msgs::JointModeCommandAction,
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
        boost::bind(&RosJointModeCommandClient::transitionCallback, this, _1));

    return mPromise->get_future();
  }
}

//==============================================================================
void RosJointModeCommandClient::transitionCallback(GoalHandle handle)
{
  int success = 0;

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
      success = result->success;
      if (!success)
      {
        message << result->message;
        ROS_WARN_STREAM_NAMED("RosJointModeCommandClient", message.str());
      }
    }
    else
    {
      // Only communication is exception
      isSuccessful = false;
    }

    if (isSuccessful)
    {
      mPromise->set_value(success);
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
void RosJointModeCommandClient::step()
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
void RosJointModeCommandClient::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning.

  mClient.cancelAllGoals();
}

} // namespace ros
} // namespace control
} // namespace aikido
