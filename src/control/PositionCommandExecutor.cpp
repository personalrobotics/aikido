#include "aikido/control/PositionCommandExecutor.hpp"

namespace aikido {
namespace control {

//==============================================================================
PositionCommandExecutor::PositionCommandExecutor(
    std::chrono::milliseconds timestep)
{
  setTimestep(timestep);
}

//==============================================================================
std::chrono::milliseconds PositionCommandExecutor::getTimestep() const
{
  return mTimestep;
}

//==============================================================================
void PositionCommandExecutor::setTimestep(std::chrono::milliseconds timestep)
{
  mTimestep = timestep;
}

} // namespace control
} // namespace aikido
