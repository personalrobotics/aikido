#include "aikido/control/ros/RosTrajectoryExecutionException.hpp"

namespace aikido {
namespace control {
namespace ros {

//==============================================================================
RosTrajectoryExecutionException::RosTrajectoryExecutionException(
    const std::string& what, actionlib::TerminalState /*terminalState*/)
  : std::runtime_error(what)
{
  // Do nothing
}

//==============================================================================
RosTrajectoryExecutionException::RosTrajectoryExecutionException(
    const std::string& what, int /*result*/)
  : std::runtime_error(what)
{
  // Do nothing
}

} // namespace ros
} // namespace control
} // namespace aikido
