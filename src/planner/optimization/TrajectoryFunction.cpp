#include "aikido/planner/optimization/TrajectoryFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
TrajectoryVariablePtr TrajectoryFunction::getTrajectoryVariable()
{
  return std::static_pointer_cast<TrajectoryVariable>(mVariable);
}

//==============================================================================
ConstTrajectoryVariablePtr TrajectoryFunction::getTrajectoryVariable() const
{
  return std::static_pointer_cast<TrajectoryVariable>(mVariable);
}

//==============================================================================
const trajectory::Trajectory&TrajectoryFunction::getTrajectory() const
{
  return getTrajectoryVariable()->getTrajectory();
}

//==============================================================================
bool TrajectoryFunction::isCompatible(const Variable& variable) const
{
  // TODO(JS): Not implemented
  return variable.isTrajectoryVariable();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
