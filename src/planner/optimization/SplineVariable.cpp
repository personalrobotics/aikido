#include "aikido/planner/optimization/SplineVariable.hpp"
#include <dart/dynamics/dynamics.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
SplineVariable::SplineVariable(const trajectory::Spline& splineToClone)
  : mSpline(
        static_cast<const trajectory::Spline&>(
            *(splineToClone.clone().release())))
{
  // Do nothing
}

//==============================================================================
const trajectory::Trajectory& SplineVariable::getTrajectory() const
{
  return static_cast<const trajectory::Trajectory&>(mSpline);
}

//==============================================================================
const trajectory::Spline& SplineVariable::getSpline() const
{
  return mSpline;
}

//==============================================================================
std::size_t SplineVariable::getDimension() const
{
  return mDimension;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
