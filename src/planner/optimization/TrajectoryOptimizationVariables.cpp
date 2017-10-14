#include "aikido/planner/optimization/TrajectoryOptimizationVariables.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
trajectory::ConstTrajectoryPtr TrajectoryOptimizationVariables::getTrajectory()
    const
{
  return mTrajectory;
}

//==============================================================================
trajectory::TrajectoryPtr TrajectoryOptimizationVariables::getTrajectory()
{
  return mTrajectory;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
