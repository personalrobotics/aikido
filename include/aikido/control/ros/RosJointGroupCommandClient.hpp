#ifndef AIKIDO_CONTROL_ROS_ROSJOINTGROUPCOMMANDCLIENT_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTGROUPCOMMANDCLIENT_HPP_

#include <chrono>
#include <future>
#include <mutex>

#include <actionlib/client/action_client.h>
#include <pr_control_msgs/JointGroupCommandAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "aikido/control/Executor.hpp"

namespace aikido {
namespace control {
namespace ros {

/// This class uses actionlib to command an action of the type
/// pr_control_msgs/JointGroupCommandAction. It specifies a set of target
/// command and sends it to the ROS server for execution
class RosJointGroupCommandClient
{
public:
  /// Constructor
  /// \param[in] node ROS node handle for action client.
  /// \param[in] serverName Name of the server to send trajectory to.
  /// \param[in] jointNames The names of the joints to set position targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosJointGroupCommandClient(
      ::ros::NodeHandle node,
      const std::string& serverName,
      std::vector<std::string> jointNames,
      std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{1000},
      std::chrono::milliseconds connectionPollingPeriod
      = std::chrono::milliseconds{20});

  virtual ~RosJointGroupCommandClient();

  /// Send command to ROS server for execution.
  /// \param[in] type Selecting between Position, Velocity, or Effort
  /// \param[in] goal Vector of target positions for each joint
  /// \param[in] timeout How long until command should expire
  std::future<int> execute(ExecutorType type, const std::vector<double> goal, ::ros::Duration timeout);

  /// To be executed on a separate thread.
  /// Regularly checks for the completion of a sent command.
  void step();

private:
  using JointGroupCommandActionClient
      = actionlib::ActionClient<pr_control_msgs::JointGroupCommandAction>;
  using GoalHandle = JointGroupCommandActionClient::GoalHandle;

  void transitionCallback(GoalHandle handle);

  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  JointGroupCommandActionClient mClient;
  JointGroupCommandActionClient::GoalHandle mGoalHandle;

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

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTGROUPCOMMANDCLIENT_HPP_
