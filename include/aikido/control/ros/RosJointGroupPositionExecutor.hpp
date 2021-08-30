#ifndef AIKIDO_CONTROL_ROS_ROSJOINTGROUPPOSITIONEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTGROUPPOSITIONEXECUTOR_HPP_

#include <chrono>
#include <future>

#include "aikido/control/PositionExecutor.hpp"
#include "aikido/control/ros/RosJointGroupCommandClient.hpp"

namespace aikido {
namespace control {
namespace ros {

/// This Executor uses pr_control_msgs/JointGrouCommandAction to
/// execute joint-wise position commands.
/// \see RosJointGroupCommandClient
class RosJointGroupPositionExecutor
  : public aikido::control::PositionExecutor
{
public:
  /// Constructor.
  /// \param[in] node ROS node handle for action client.
  /// \param[in] controllerName Name of the controller to send command to.
  /// \param[in] jointNames The names of the joints to set goal targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosJointGroupPositionExecutor(
      ::ros::NodeHandle node,
      const std::string& controllerName,
      std::vector<std::string> jointNames,
      std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{1000},
      std::chrono::milliseconds connectionPollingPeriod
      = std::chrono::milliseconds{20});

  virtual ~RosJointGroupPositionExecutor();

  /// \copydoc PositionExecutor::execute()
  std::future<int> execute(const std::vector<double> command, const std::chrono::duration<double>& timeout) override;

  /// \copydoc Executor::step()
  void step(const std::chrono::system_clock::time_point& timepoint) override;

private:

  RosJointGroupCommandClient mClient;
};

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTGROUPPOSITIONEXECUTOR_HPP_
