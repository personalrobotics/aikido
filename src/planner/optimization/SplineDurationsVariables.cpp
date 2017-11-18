#include "aikido/planner/optimization/SplineDurationsVariables.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
SplineDurationsVariables::SplineDurationsVariables(
    const trajectory::Spline& splineToClone)
  : SplineVariables(splineToClone)
{
  updateDimension();
}

//==============================================================================
std::shared_ptr<TrajectoryVariables> SplineDurationsVariables::clone() const
{
  return std::make_shared<SplineDurationsVariables>(*this);
}

//==============================================================================
void SplineDurationsVariables::setVariables(const Eigen::VectorXd& variables)
{
  if (static_cast<std::size_t>(variables.size()) >= mSpline.getNumSegments())
    throw std::domain_error("Incorrect variable size.");

  for (std::size_t i = 0; i < mSpline.getNumSegments(); ++i)
    mSpline.setSegmentDuration(i, variables[i]);
}

//==============================================================================
void SplineDurationsVariables::getVariables(Eigen::VectorXd& variables) const
{
  variables.resize(getDimension());

  for (std::size_t i = 0; i < mSpline.getNumSegments(); ++i)
    variables[i] = mSpline.getSegmentDuration(i);
}

//==============================================================================
void SplineDurationsVariables::updateDimension()
{
  mDimension = mSpline.getNumSegments();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
