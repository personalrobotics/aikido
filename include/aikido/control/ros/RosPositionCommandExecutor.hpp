#ifndef AIKIDO_CONTROL_ROS_ROSPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSPOSITIONCOMMANDEXECUTOR_HPP_
#include <chrono>
#include <future>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>
#include <pr_control_msgs/SetPositionAction.h>
#include <actionlib/client/action_client.h>
#include <aikido/control/PositionCommandExecutor.hpp>
#include <Eigen/Dense>

namespace aikido{
namespace control {
namespace ros {

/// This class uses actionlib to command an action of the type
/// pr_control_msgs/SetPosition. It specifies a set of target
/// positions and sends it to the ROS server for execution 
class RosPositionCommandExecutor : public aikido::control::PositionCommandExecutor
{
public:
  /// Constructor
  /// \param[in] node ROS node handle for action client.
  /// \param[in] serverName Name of the server to send trajectory to.
  /// \param[in] The names of the joints to set position targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosPositionCommandExecutor(
    ::ros::NodeHandle node,
    const std::string& serverName,
    std::vector<std::string> jointNames,
    std::chrono::milliseconds connectionTimeout = std::chrono::milliseconds{1000},
    std::chrono::milliseconds connectionPollingPeriod = std::chrono::milliseconds{20}
  );

  virtual ~RosPositionCommandExecutor();

  /// Sends positions to ROS server for execution.
  /// \param[in] goalPositions Vector of target positions for each joint
  std::future<void> execute(const Eigen::VectorXd& goalPositions) override;

  /// \copydoc PositionCommandExecutor::step()
  ///
  /// To be executed on a separate thread.
  /// Regularly checks for the completion of a sent trajectory.
  void step() override;

private:
  using RosPositionActionClient
    = actionlib::ActionClient<pr_control_msgs::SetPositionAction>;
  using GoalHandle = RosPositionActionClient::GoalHandle;

  bool waitForServer();

  void transitionCallback(GoalHandle handle);

  ::ros::NodeHandle mNode;
  ::ros::CallbackQueue mCallbackQueue;
  RosPositionActionClient mClient;
  RosPositionActionClient::GoalHandle mGoalHandle;

  std::chrono::milliseconds mConnectionTimeout;
  std::chrono::milliseconds mConnectionPollingPeriod;

  bool mInProgress;
  std::promise<void> mPromise;
  Eigen::VectorXd mGoalPositions;
  std::vector<std::string> mJointNames;

  std::mutex mMutex;
};
    
} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSPOSITIONCOMMANDEXECUTOR_HPP_
