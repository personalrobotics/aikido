#ifndef AIKIDO_CONTROL_ROS_ROSJOINTMODECOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_ROS_ROSJOINTMODECOMMANDEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include "aikido/control/JointModeCommandExecutor.hpp"
#include "aikido/control/ros/RosJointModeCommandClient.hpp"

namespace aikido {
namespace control {
namespace ros {

/// Default actionlib client connection timeout (ms)
constexpr int DEFAULT_CON_TIMEOUT_MS{1000};

/// Default actionlib client polling period (ms)
constexpr int DEFAULT_POLL_PERIOD_MS{20};

/// This Executor uses pr_control_msgs/JointModeCommandAction to
/// switch to required joint wise mode targets.
/// \see RosJointModeCommandClient
class RosJointModeCommandExecutor : public aikido::control::JointModeCommandExecutor
{
public:
  /// Constructor.
  /// \param[in] node ROS node handle for action client.
  /// \param[in] controllerName Name of the controller to send command to.
  /// \param[in] dofs The Degrees of Freedom to set joint mode targets for
  /// \param[in] connectionTimeout Timeout for server connection.
  /// \param[in] connectionPollingPeriod Polling period for server connection.
  RosJointModeCommandExecutor(
      ::ros::NodeHandle node,
      const std::string& controllerName, // Rajat TODO: Remove this as controller name is fixed?
      const std::vector<dart::dynamics::DegreeOfFreedom*>& dofs,
      std::chrono::milliseconds connectionTimeout
      = std::chrono::milliseconds{DEFAULT_CON_TIMEOUT_MS},
      std::chrono::milliseconds connectionPollingPeriod
      = std::chrono::milliseconds{DEFAULT_POLL_PERIOD_MS});

  virtual ~RosJointModeCommandExecutor();

  // Documentation inherited.
  std::future<int> execute(
      const std::vector<hardware_interface::JointCommandModes>& command,
      const std::chrono::duration<double>& timeout,
      const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  void cancel() override;

private:
  RosJointModeCommandClient mClient;
};

} // namespace ros
} // namespace control
} // namespace aikido

#include "detail/RosJointModeCommandExecutor-impl.hpp"

#endif // ifndef AIKIDO_CONTROL_ROS_ROSJOINTMODECOMMANDEXECUTOR_HPP_
