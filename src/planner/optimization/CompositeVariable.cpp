#include "aikido/planner/optimization/CompositeVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
CompositeVariable::Index CompositeVariable::InvalidIndex
    = std::numeric_limits<std::size_t>::max();

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
    variable->setValue(value.segment(segmentIndex, variable->getDimension()));
    segmentIndex += variable->getDimension();
  }
}

//==============================================================================
Eigen::VectorXd CompositeVariable::getValue() const
{
  Eigen::VectorXd value(getDimension());

  std::size_t segmentIndex = 0u;
  for (auto& variable : mVariables)
  {
    value.segment(segmentIndex, variable->getDimension())
        = variable->getValue();
    segmentIndex += variable->getDimension();
  }

  return value;
}

//==============================================================================
Eigen::Map<const Eigen::VectorXd> CompositeVariable::getValueSegment(
    const Eigen::VectorXd& value, const Variable* variable) const
{
  Eigen::Map<const Eigen::VectorXd> map(
      value.data() + getSubVariableIndex(variable), variable->getDimension());

  return map;
}

//==============================================================================
std::size_t CompositeVariable::addSubVariable(VariablePtr variable)
{
  // TODO(JS): Should we check duplicity?

  mVariables.emplace_back(std::move(variable));

  return mVariables.size() - 1u;
}

//==============================================================================
ConstVariablePtr CompositeVariable::getSubVariable(std::size_t index) const
{
  // TODO(JS): Check index validity
  return mVariables[index];
}

//==============================================================================
std::size_t CompositeVariable::getSubVariableIndex(
    const Variable* variable) const
{
  auto result = std::find_if(
      mVariables.begin(),
      mVariables.end(),
      [&variable](const VariablePtr& var) { return var.get() == variable; });

  if (result == mVariables.end())
    return InvalidIndex;
  else
    return std::distance(result, mVariables.begin());
}

//==============================================================================
void CompositeVariable::updateDimension()
{
  mDimension = 0u;
  for (const auto& variable : mVariables)
    mDimension += variable->getDimension();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
