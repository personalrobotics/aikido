#include "aikido/planner/optimization/CompositeFunction.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
bool CompositeFunction::isCompatible(const Variable& /*variable*/) const
{
  // TODO(JS): Return false if variable is not a CompositeVariable.

  return true;
}

//==============================================================================
double CompositeFunction::eval(const Eigen::VectorXd& x)
{
  double value = 0.0;
  std::size_t segmentIndex = 0;

  for (auto& functionAndIndex : mFunctionToVariable)
  {
    auto& function = functionAndIndex.first;
    auto& subVariableIndex = functionAndIndex.second;
    auto subVariable = getCompositeVariable()->getSubVariable(subVariableIndex);
    const auto segmentSize = subVariable->getDimension();

    value += function->eval(x.segment(segmentIndex, segmentSize));
    segmentIndex += segmentSize;
  }

  assert(segmentIndex == static_cast<std::size_t>(x.size()));

  return value;
}

//==============================================================================
void CompositeFunction::addFunction(FunctionPtr function, VariablePtr variable)
{
  auto subVariableIndex
      = getCompositeVariable()->getSubVariableIndex(variable.get());

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
