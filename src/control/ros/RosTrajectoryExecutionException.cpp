#include <aikido/control/ros/RosTrajectoryExecutionException.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/statespace/dart/SO2Joint.hpp>
#include <aikido/util/StepSequence.hpp>

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
