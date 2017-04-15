// Assume it has 4 positions
#ifndef AIKIDO_CONTROL_ROS_ROSBARRETTHANDPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSBARRETTHANDPOSITIONCOMMANDEXECUTOR_HPP_
#include <chrono>
#include <future>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <pr_control_msgs/SetPositionAction.h>
#include <actionlib/client/action_client.h>
#include <Eigen/Dense>

namespace aikido{
namespace control {
namespace ros {

/// This class sends commands for the hand position to the ROS server
class RosBarrettHandPositionCommandExecutor
{
public:
  /// Constructor
  /// \param[in] node ROS node handle for action client.
  /// \param[in] serverName Name of the server to send traejctory to.
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  template <typename DurationA, typename DurationB>
  RosBarrettHandPositionCommandExecutor(
    ::ros::NodeHandle node,
    const std::string& serverName,
    const DurationA& connectionTimeout = std::chrono::milliseconds{1000},
    const DurationB& connectionPollingPeriod = std::chrono::milliseconds{20}
  );

  ~RosBarrettHandPositionCommandExecutor();

  /// Sends positions to ROS server for execution.
  /// \param[in] goalPositions Vector of 3 finger positions and 1 spread position
  /// \param[in] jointNames Vector of 4 joint names for fingers
  std::future<void> execute(Eigen::VectorXd& goalPositions,std::vector<std::string>& jointNames);

  /// Sends positions to ROS server for execution.
  /// \param[in] goalPositions Vector of 3 finger positions and 1 spread position
  /// \param[in] jointNames Vector of 4 joint names for fingers
  /// \param[in] startTime Start time for the trajectory.
  std::future<void> execute(
    Eigen::VectorXd& goalPositions, 
    std::vector<std::string>& jointNames,
    const ::ros::Time& startTime);

  void step();

private:
  using HandPositionActionClient
    = actionlib::ActionClient<pr_control_msgs::SetPositionAction>;
  using GoalHandle = HandPositionActionClient::GoalHandle;

  bool waitForServer();

  void transitionCallback(GoalHandle handle);

  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  HandPositionActionClient mClient;
  HandPositionActionClient::GoalHandle mGoalHandle;

  std::chrono::milliseconds mConnectionTimeout;
  std::chrono::milliseconds mConnectionPollingPeriod;

  bool mInProgress;
  std::promise<void> mPromise;
  Eigen::VectorXd mGoalPositions;

  std::mutex mMutex;
};
    
} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSBARRETTHANDPOSITIONCOMMANDEXECUTOR_HPP_