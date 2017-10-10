#include "aikido/planner/optimization/SplineDurationsVariables.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
SplineDurationsVariables::SplineDurationsVariables(
    statespace::StateSpacePtr sspace, double startTime)
  : trajectory::Trajectory(), Spline(sspace, startTime)
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<OptimizationVariables> SplineDurationsVariables::clone() const
{
  auto cloned = std::make_shared<SplineDurationsVariables>(*this);

  return cloned;
}

//==============================================================================
std::size_t SplineDurationsVariables::getDimension() const
{
  return mSegments.size();
}

//==============================================================================
void SplineDurationsVariables::setVariables(const Eigen::VectorXd& variables)
{
  // TODO(JS): Check the dimension of variables

  for (std::size_t i = 0; i < mSegments.size(); ++i)
    mSegments[i].mDuration = variables[i];
}

//==============================================================================
void SplineDurationsVariables::getVariables(Eigen::VectorXd& variables) const
{
  variables.resize(getDimension());

  for (std::size_t i = 0; i < mSegments.size(); ++i)
    variables[i] = mSegments[i].mDuration;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
