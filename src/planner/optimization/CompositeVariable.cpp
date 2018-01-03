#include "aikido/planner/optimization/CompositeVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
CompositeVariable::Index CompositeVariable::InvalidIndex
    = std::numeric_limits<std::size_t>::max();

//==============================================================================
CompositeVariable::CompositeVariable() : mNeedDimensionUpdate(true)
{
  // Do nothing
}

//==============================================================================
std::size_t CompositeVariable::getDimension() const
{
  if (mNeedDimensionUpdate)
    updateDimension();

  return mDimension;
}

//==============================================================================
UniqueVariablePtr CompositeVariable::clone() const
{
  return dart::common::make_unique<CompositeVariable>();
}

//==============================================================================
void CompositeVariable::setValue(const Eigen::VectorXd& value)
{
  if (static_cast<std::size_t>(value.size()) != getDimension())
  {
    throw std::invalid_argument(
        "Inconsistent dimension between CompositeVariable and value");
  }

  std::size_t segmentIndex = 0u;
  for (auto& variable : mSubVariables)
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
  for (auto& variable : mSubVariables)
  {
    value.segment(segmentIndex, variable->getDimension())
        = variable->getValue();
    segmentIndex += variable->getDimension();
  }

  return value;
}

//==============================================================================
Eigen::Map<const Eigen::VectorXd> CompositeVariable::getSubValue(
    const Eigen::VectorXd& value, const Variable* variable) const
{
  Eigen::Map<const Eigen::VectorXd> map(
      value.data() + getSubVariableIndex(variable), variable->getDimension());

  return map;
}

//==============================================================================
std::size_t CompositeVariable::addSubVariable(VariablePtr variable)
{
  auto result = std::find(mSubVariables.begin(), mSubVariables.end(), variable);
  if (result != mSubVariables.end())
    return std::distance(mSubVariables.begin(), result);

  mSubVariables.emplace_back(std::move(variable));

  mNeedDimensionUpdate = true;

  return mSubVariables.size() - 1u;
}

//==============================================================================
ConstVariablePtr CompositeVariable::getSubVariable(std::size_t index) const
{
  // TODO(JS): Check index validity
  return mSubVariables[index];
}

//==============================================================================
std::size_t CompositeVariable::getSubVariableIndex(
    const Variable* variable) const
{
  auto result = std::find_if(
      mSubVariables.begin(),
      mSubVariables.end(),
      [&variable](const VariablePtr& var) { return var.get() == variable; });

  if (result == mSubVariables.end())
    return InvalidIndex;
  else
    return std::distance(mSubVariables.begin(), result);
}

//==============================================================================
bool CompositeVariable::hasSubVariable(const Variable* variable) const
{
  auto result = std::find_if(
      mSubVariables.begin(),
      mSubVariables.end(),
      [&variable](const VariablePtr& var) { return var.get() == variable; });

  if (result == mSubVariables.end())
    return false;
  else
    return true;
}

//==============================================================================
void CompositeVariable::updateDimension() const
{
  mDimension = 0u;
  for (const auto& variable : mSubVariables)
    mDimension += variable->getDimension();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
