#include <aikido/control/TrajectoryRunningException.hpp>

namespace aikido {
namespace control {

//==============================================================================
TrajectoryRunningException::TrajectoryRunningException()
  : std::runtime_error("Another trajectory is in progress.")
{
  // Do nothing
}

} // namespace control
} // namespace aikido
