#include "aikido/planner/optimization/CompositeFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
CompositeFunction::CompositeFunction(VariablePtr variable)
  : Function(std::move(variable))
{
  // Do nothing
}

//==============================================================================
CompositeFunction::~CompositeFunction()
{
  // Do nothing
}

//==============================================================================
UniqueFunctionPtr CompositeFunction::clone() const
{
  return dart::common::make_unique<CompositeFunction>(*this);
}

//==============================================================================
bool CompositeFunction::isCompatible(const Variable& variable) const
{
  if (!dynamic_cast<const CompositeVariable*>(&variable))
    return false;

  return true;
}

//==============================================================================
double CompositeFunction::eval(const Eigen::VectorXd& x)
{
  if (static_cast<std::size_t>(x.size()) != getDimension())
  {
    throw std::invalid_argument(
        "Inconsistent dimension between CompositeVariable and value");
  }

  double value = 0.0;

  for (auto& functionAndIndex : mFunctionToVariable)
  {
    auto& function = functionAndIndex.first;
    auto& subVariableIndex = functionAndIndex.second;
    auto subVariable = getCompositeVariable()->getSubVariable(subVariableIndex);
    const auto segmentSize = subVariable->getDimension();

    value += function->eval(x.segment(subVariableIndex, segmentSize));
  }

  return value;
}

//==============================================================================
void CompositeFunction::addSubFunction(
    FunctionPtr function, VariablePtr variable)
{
  auto subVariableIndex
      = getCompositeVariable()->getSubVariableIndex(variable.get());

  if (subVariableIndex == CompositeVariable::InvalidIndex)
  {
    throw std::invalid_argument(
        "variable is not an associated sub variable to this function.");
  }

  mFunctionToVariable.insert(
      std::make_pair(std::move(function), subVariableIndex));
}

//==============================================================================
CompositeVariablePtr CompositeFunction::getCompositeVariable()
{
  return std::static_pointer_cast<CompositeVariable>(mVariable);
}

} // namespace optimization
} // namespace planner
} // namespace aikido
