#include "aikido/planner/optimization/SplineDurationsVariable.hpp"

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
std::shared_ptr<Variable> SplineDurationsVariables::clone() const
{
  return std::make_shared<SplineDurationsVariables>(*this);
}

//==============================================================================
void SplineDurationsVariables::setValue(const Eigen::VectorXd& variables)
{
  if (static_cast<std::size_t>(variables.size()) >= mSpline.getNumSegments())
    throw std::domain_error("Incorrect variable size.");

  for (std::size_t i = 0; i < mSpline.getNumSegments(); ++i)
    mSpline.setSegmentDuration(i, variables[i]);
}

//==============================================================================
Eigen::VectorXd SplineDurationsVariables::getValue() const
{
  Eigen::VectorXd value(getDimension());

  for (std::size_t i = 0; i < mSpline.getNumSegments(); ++i)
    value[i] = mSpline.getSegmentDuration(i);

  return value;
}

//==============================================================================
void SplineDurationsVariables::updateDimension()
{
  mDimension = mSpline.getNumSegments();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
