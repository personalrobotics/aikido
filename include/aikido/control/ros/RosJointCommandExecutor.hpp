#ifndef AIKIDO_CONTROL_ROS_ROSJOINTCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTCOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include "aikido/control/JointCommandExecutor.hpp"
#include "aikido/control/ros/RosJointGroupCommandClient.hpp"

namespace aikido {
namespace control {
namespace ros {

/// Default actionlib client connection timeout (ms)
constexpr int DEFAULT_CON_TIMEOUT_MS{1000};

/// Default actionlib client polling period (ms)
constexpr int DEFAULT_POLL_PERIOD_MS{20};

/// This Executor uses pr_control_msgs/JointGroupCommandAction to
/// execute joint-wise commands.
/// \see RosJointGroupCommandClient
template <ExecutorType T>
class RosJointCommandExecutor : public aikido::control::JointCommandExecutor<T>
{
public:
  /// Constructor.
  /// \param[in] node ROS node handle for action client.
  /// \param[in] controllerName Name of the controller to send command to.
  /// \param[in] dofs The Degrees of Freedom to set goal targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosJointCommandExecutor(
      ::ros::NodeHandle node,
      const std::string& controllerName,
      const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
      std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{DEFAULT_CON_TIMEOUT_MS},
      std::chrono::milliseconds connectionPollingPeriod
      = std::chrono::milliseconds{DEFAULT_POLL_PERIOD_MS});

  virtual ~RosJointCommandExecutor();

  // Documentation inherited.
  std::future<int> execute(
      const std::vector<double>& command,
      const std::chrono::duration<double>& timeout,
      const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  void cancel() override;

private:
  RosJointGroupCommandClient mClient;
};

// Define common executors

using RosJointPositionExecutor
    = RosJointCommandExecutor<ExecutorType::POSITION>;
using RosJointVelocityExecutor
    = RosJointCommandExecutor<ExecutorType::VELOCITY>;
using RosJointEffortExecutor = RosJointCommandExecutor<ExecutorType::EFFORT>;

} // namespace ros
} // namespace control
} // namespace aikido

#include "detail/RosJointCommandExecutor-impl.hpp"

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTCOMMANDEXECUTOR_HPP_
