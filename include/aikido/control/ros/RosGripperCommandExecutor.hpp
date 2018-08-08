#ifndef AIKIDO_CONTROL_ROS_ROSGRIPPERCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSGRIPPERCOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <mutex>
#include <Eigen/Dense>
#include <actionlib/client/action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace aikido {
namespace control {
namespace ros {

/// This class uses actionlib to command an action of the type
/// pr_control_msgs/SetPosition. It specifies a set of target
/// positions and sends it to the ROS server for execution
class RosGripperCommandExecutor
{
public:
  /// Constructor
  /// \param[in] node ROS node handle for action client.
  /// \param[in] serverName Name of the server to send trajectory to.
  /// \param[in] jointNames The names of the joints to set position targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosGripperCommandExecutor(
      ::ros::NodeHandle node,
      const std::string& serverName,
      std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{1000},
      std::chrono::milliseconds connectionPollingPeriod
      = std::chrono::milliseconds{20});

  virtual ~RosGripperCommandExecutor();

  /// Sends positions to ROS server for execution.
  /// \param[in] goalPositions Vector of target positions for each joint
  std::future<void> execute(const double goalPosition, const double effort);

  /// \copydoc PositionCommandExecutor::step()
  ///
  /// To be executed on a separate thread.
  /// Regularly checks for the completion of a sent trajectory.
  void step(const std::chrono::system_clock::time_point& timepoint);

private:
  using RosGripperActionClient
      = actionlib::ActionClient<control_msgs::GripperCommandAction>;
  using GoalHandle = RosGripperActionClient::GoalHandle;

  void transitionCallback(GoalHandle handle);

  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  RosGripperActionClient mClient;
  RosGripperActionClient::GoalHandle mGoalHandle;

  std::vector<std::string> mJointNames;

  std::chrono::milliseconds mConnectionTimeout;
  std::chrono::milliseconds mConnectionPollingPeriod;

  bool mInProgress;
  std::unique_ptr<std::promise<void>> mPromise;

  /// Manages access to mInProgress, mPromise
  mutable std::mutex mMutex;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSPOSITIONCOMMANDEXECUTOR_HPP_
