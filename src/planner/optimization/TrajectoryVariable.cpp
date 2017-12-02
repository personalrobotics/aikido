#include "aikido/planner/optimization/TrajectoryVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
bool TrajectoryVariable::isTrajectoryVariable() const
{
  return true;
}

//==============================================================================
TrajectoryVariable* TrajectoryVariable::asTrajectoryVariable()
{
  return this;
}

//==============================================================================
const TrajectoryVariable* TrajectoryVariable::asTrajectoryVariable() const
{
  return this;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
