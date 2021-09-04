#ifndef AIKIDO_CONTROL_ROS_ROSJOINTCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTCOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>

#include "aikido/control/JointCommandExecutor.hpp"
#include "aikido/control/ros/RosJointGroupCommandClient.hpp"

namespace aikido {
namespace control {
namespace ros {

/// This Executor uses pr_control_msgs/JointGrouCommandAction to
/// execute joint-wise velocity commands.
/// \see RosJointGroupCommandClient
template<ExecutorType T>
class RosJointCommandExecutor : public aikido::control::JointCommandExecutor<T>
{
public:
  /// Constructor.
  /// \param[in] node ROS node handle for action client.
  /// \param[in] controllerName Name of the controller to send command to.
  /// \param[in] jointNames The names of the joints to set goal targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosJointCommandExecutor(
      ::ros::NodeHandle node,
      const std::string& controllerName,
      std::vector<std::string> jointNames,
      std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{1000},
      std::chrono::milliseconds connectionPollingPeriod
      = std::chrono::milliseconds{20});

  virtual ~RosJointCommandExecutor();

  /// Documentation inherited.
  std::future<int> execute(
      const std::vector<double> command,
      const std::chrono::duration<double>& timeout) override;

  /// Documentation inherited.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  /// Documentation inherited.
  void cancel() override;

private:
  RosJointGroupCommandClient mClient;
};

// Define common executors

using RosJointPositionExecutor = RosJointCommandExecutor<ExecutorType::POSITION>;
using RosJointVelocityExecutor = RosJointCommandExecutor<ExecutorType::VELOCITY>;
using RosJointEffortExecutor = RosJointCommandExecutor<ExecutorType::EFFORT>;

} // namespace ros
} // namespace control
} // namespace aikido

#include "detail/RosJointCommandExecutor-impl.hpp"

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTCOMMANDEXECUTOR_HPP_
