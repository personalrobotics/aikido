#include "aikido/control/TrajectoryExecutor.hpp"

namespace aikido {
namespace control {

//==============================================================================
TrajectoryExecutor::TrajectoryExecutor(std::chrono::milliseconds timestep)
{
  setTimestep(timestep);
}

//==============================================================================
std::chrono::milliseconds TrajectoryExecutor::getTimestep() const
{
  return mTimestep;
}

//==============================================================================
void TrajectoryExecutor::setTimestep(std::chrono::milliseconds timestep)
{
  mTimestep = timestep;
}

} // namespace control
} // namespace aikido
