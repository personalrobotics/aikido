#include "aikido/control/ros/RosJointGroupEffortExecutor.hpp"

namespace aikido {
namespace control {
namespace ros {

using std::chrono::milliseconds;

//==============================================================================
RosJointGroupEffortExecutor::RosJointGroupEffortExecutor(
    ::ros::NodeHandle node,
    const std::string& controllerName,
    std::vector<std::string> jointNames,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  : EffortExecutor(jointNames)
  , mClient{node,
            controllerName + "/joint_group_command",
            jointNames,
            connectionTimeout,
            connectionPollingPeriod}
{
  // Do nothing.
}

//==============================================================================
RosJointGroupEffortExecutor::~RosJointGroupEffortExecutor()
{
  // Do nothing.
}

//==============================================================================
std::future<int> RosJointGroupEffortExecutor::execute(
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{
  ::ros::Duration duration;
  duration.sec
      = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
  duration.nsec
      = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count()
        % 1000000000UL;
  return mClient.execute(ExecutorType::kEFFORT, command, duration);
}

//==============================================================================
void RosJointGroupEffortExecutor::step(
    const std::chrono::system_clock::time_point& /* timepoint */)
{
  mClient.step();
}

//==============================================================================
void RosJointGroupEffortExecutor::cancel()
{
  mClient.cancel();
}

} // namespace ros
} // namespace control
} // namespace aikido
