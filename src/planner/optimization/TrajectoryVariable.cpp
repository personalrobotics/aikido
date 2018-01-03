#include "aikido/planner/optimization/TrajectoryVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
TrajectoryVariable::TrajectoryVariable() : mNeedDimensionUpdate(true)
{
  // Do nothing
}

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

//==============================================================================
std::size_t TrajectoryVariable::getDimension() const
{
  if (mNeedDimensionUpdate)
    updateDimension();

  return mDimension;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
