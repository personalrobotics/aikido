#include "aikido/planner/optimization/TrajectoryOptimizationVariables.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
trajectory::Trajectory* TrajectoryOptimizationVariables::getTrajectory() const
{
  return mTrajectory.get();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
