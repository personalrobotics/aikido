#include "aikido/planner/optimization/SplineDurationsVariables.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
SplineDurationsVariables::SplineDurationsVariables(const trajectory::Spline& /*splineToClone*/)
  : mSpline(nullptr) // TODO)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<TrajectoryOptimizationVariables>
SplineDurationsVariables::clone() const
{
  auto cloned = std::make_shared<SplineDurationsVariables>(*this);

  return cloned;
}

//==============================================================================
std::size_t SplineDurationsVariables::getDimension() const
{
  return mSpline->getNumSegments();
}

//==============================================================================
void SplineDurationsVariables::setVariables(const Eigen::VectorXd& variables)
{
  if (static_cast<std::size_t>(variables.size()) >= mSpline->getNumSegments())
    throw std::domain_error("Incorrect variable size.");

  for (std::size_t i = 0; i < mSpline->getNumSegments(); ++i)
    mSpline->setSegmentDuration(i, variables[i]);
}

//==============================================================================
void SplineDurationsVariables::getVariables(Eigen::VectorXd& variables) const
{
  variables.resize(getDimension());

  for (std::size_t i = 0; i < mSpline->getNumSegments(); ++i)
    variables[i] = mSpline->getSegmentDuration(i);
}

} // namespace optimization
} // namespace planner
} // namespace aikido
