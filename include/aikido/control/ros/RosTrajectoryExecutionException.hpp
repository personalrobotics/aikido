#ifndef AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTIONEXCEPTION_HPP_
#define AIKIDO_CONTROL_ROS_ROSTRAJECTORYEXECUTIONEXCEPTION_HPP_
#include <exception>
#include <actionlib/client/action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace aikido {
namespace control {
namespace ros {

/// This class wraps various exceptions that may arise during trajectory
/// execution over ROS.
class RosTrajectoryExecutionException: public std::runtime_error
{
public:

  RosTrajectoryExecutionException(
    const std::string& what,
    actionlib::TerminalState terminalState);

  RosTrajectoryExecutionException(
    const std::string& what,
    int result);

  virtual ~RosTrajectoryExecutionException() = default;

};

} // ros
} // control
} // aikido

#endif
