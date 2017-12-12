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

//==============================================================================
Eigen::VectorXd Variable::createValue() const
{
  return Eigen::VectorXd(getDimension());
}

//==============================================================================
Eigen::VectorXd Variable::createValue(double value) const
{
  return Eigen::VectorXd::Constant(getDimension(), value);
}

//==============================================================================
Eigen::VectorXd Variable::createZeroValue() const
{
  return Eigen::VectorXd::Zero(getDimension());
}

} // namespace optimization
} // namespace planner
} // namespace aikido
