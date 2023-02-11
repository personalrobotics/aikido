#ifndef AIKIDO_CONTROL_ROS_ROSJOINTMODECOMMANDCLIENT_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTMODECOMMANDCLIENT_HPP_

#include <chrono>
#include <future>
#include <mutex>

#include <actionlib/client/action_client.h>
#include <pr_control_msgs/JointGroupCommandAction.h>
#include <pr_control_msgs/JointModeCommandAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

// ros_control
#include <hardware_interface/joint_mode_interface.h>

namespace aikido {
namespace control {
namespace ros {

static inline int intFromMode(hardware_interface::JointCommandModes mode)
{
  switch (mode)
  {
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
      ROS_WARN_STREAM("Setting error mode to'" << 6 << "'.");
      return 6;
  }
}

/// This class uses actionlib to command an action of the type
/// pr_control_msgs/JointModeCommandAction. It specifies a target joint mode
/// command and sends it to the ROS server for setting
class RosJointModeCommandClient
{
public:
  /// Constructor
  /// \param[in] node ROS node handle for action client.
  /// \param[in] serverName Name of the server to send mode target to.
  /// \param[in] jointNames The names of the joints to set mode targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosJointModeCommandClient(
      const ::ros::NodeHandle& node,
      const std::string& serverName,
      const std::vector<std::string> jointNames,
      const std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{1000},
      const std::chrono::milliseconds connectionPollingPeriod
      = std::chrono::milliseconds{20});

  virtual ~RosJointModeCommandClient();

  /// Send command to ROS server for execution.
  /// \param[in] goal target joint command mode of target positions for each
  /// joint
  /// \param[in] timeout How long until command should expire
  std::future<int> execute(
      const std::vector<hardware_interface::JointCommandModes>& goal);

  /// \copydoc JointModeCommandExecutor::step()
  /// To be executed on a separate thread.
  /// Regularly checks for the completion of a sent command.
  void step();

  /// Cancel current command.
  /// Should trigger exception for active std::promise
  void cancel();

private:
  using JointModeCommandActionClient
      = actionlib::ActionClient<pr_control_msgs::JointModeCommandAction>;
  using GoalHandle = JointModeCommandActionClient::GoalHandle;

  void transitionCallback(GoalHandle handle);

  const ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  JointModeCommandActionClient mClient;
  JointModeCommandActionClient::GoalHandle mGoalHandle;

  std::vector<std::string> mJointNames;

  std::chrono::milliseconds mConnectionTimeout;
  std::chrono::milliseconds mConnectionPollingPeriod;

  bool mInProgress;
  std::unique_ptr<std::promise<int>> mPromise;

  /// Manages access to mInProgress, mPromise
  mutable std::mutex mMutex;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTMODECOMMANDCLIENT_HPP_
