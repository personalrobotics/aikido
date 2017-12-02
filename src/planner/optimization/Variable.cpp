#include "aikido/planner/optimization/Variable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
bool Variable::isTrajectoryVariable() const
{
  return false;
}

//==============================================================================
TrajectoryVariable* Variable::asTrajectoryVariable()
{
  return nullptr;
}

//==============================================================================
const TrajectoryVariable* Variable::asTrajectoryVariable() const
{
  return nullptr;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
