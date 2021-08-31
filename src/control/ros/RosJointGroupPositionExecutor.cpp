#include "aikido/control/ros/RosJointGroupPositionExecutor.hpp"

namespace aikido {
namespace control {
namespace ros {

using std::chrono::milliseconds;

//==============================================================================
RosJointGroupPositionExecutor::RosJointGroupPositionExecutor(
    ::ros::NodeHandle node,
    const std::string& controllerName,
    std::vector<std::string> jointNames,
    const std::chrono::milliseconds connectionTimeout,
    const std::chrono::milliseconds connectionPollingPeriod)
  : PositionExecutor(jointNames)
  , mClient{node,
            controllerName + "/joint_group_command",
            jointNames,
            connectionTimeout,
            connectionPollingPeriod}
{
  // Do nothing.
}

//==============================================================================
RosJointGroupPositionExecutor::~RosJointGroupPositionExecutor()
{
  stop();
}

//==============================================================================
std::future<int> RosJointGroupPositionExecutor::execute(
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{
  ::ros::Duration duration;
  duration.sec
      = std::chrono::duration_cast<std::chrono::seconds>(timeout).count();
  duration.nsec
      = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count()
        % 1000000000UL;
  return mClient.execute(ExecutorType::kPOSITION, command, duration);
}

//==============================================================================
void RosJointGroupPositionExecutor::step(
    const std::chrono::system_clock::time_point& /* timepoint */)
{
  mClient.step();
}

//==============================================================================
void RosJointGroupPositionExecutor::cancel()
{
  mClient.cancel();
}

} // namespace ros
} // namespace control
} // namespace aikido
