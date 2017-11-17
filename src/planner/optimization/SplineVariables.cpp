#include "aikido/planner/optimization/SplineVariables.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
const trajectory::Spline*SplineVariables::getSpline() const
{
  assert(dynamic_cast<trajectory::Spline*>(getTrajectory()));
  return static_cast<trajectory::Spline*>(getTrajectory());
}

//==============================================================================
trajectory::Spline*SplineVariables::getSpline()
{
  assert(dynamic_cast<trajectory::Spline*>(getTrajectory()));
  return static_cast<trajectory::Spline*>(getTrajectory());
}

} // namespace optimization
} // namespace planner
} // namespace aikido
