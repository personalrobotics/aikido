#include "aikido/planner/optimization/CompositeVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
std::unique_ptr<Variable> CompositeVariable::clone() const
{
  return dart::common::make_unique<CompositeVariable>();
}

//==============================================================================
std::size_t CompositeVariable::getDimension() const
{
  return mDimension;
}

//==============================================================================
void CompositeVariable::setValue(const Eigen::VectorXd& value)
{
  std::size_t segmentIndex = 0u;
  for (auto& variable : mVariables)
  {
    variable.setValue(value.segment(segmentIndex, variable.getDimension()));
    segmentIndex += variable.getDimension();
  }
}

//==============================================================================
Eigen::VectorXd CompositeVariable::getValue() const
{
  Eigen::VectorXd value(getDimension());

  std::size_t segmentIndex = 0u;
  for (auto& variable : mVariables)
  {
    value.segment(segmentIndex, variable.getDimension()) = variable.getValue();
    segmentIndex += variable.getDimension();
  }

  return value;
}

//==============================================================================
void CompositeVariable::addVariable(const Variable& variableToClone)
{
  variableToClone.clone();
//  mVariables.emplace_back();
}

//==============================================================================
void CompositeVariable::updateDimension()
{
  mDimension = 0u;
  for (const auto& variable : mVariables)
    mDimension += variable.getDimension();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
