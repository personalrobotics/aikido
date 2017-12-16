#include "aikido/planner/optimization/Function.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
Function::Function(VariablePtr variable) : mVariable(std::move(variable))
{
  if (!variable)
    throw std::invalid_argument("nullptr variable is not allowed");
}

//==============================================================================
void Function::setVariable(VariablePtr variable)
{
  if (variable == mVariable)
    return;

  if (variable and !isCompatible(*variable))
    throw std::invalid_argument("Invalid variable for this function.");

  mVariable = std::move(variable);
}

//==============================================================================
ConstVariablePtr Function::getVariable() const
{
  return mVariable;
}

//==============================================================================
std::size_t Function::getDimension() const
{
  return mVariable->getDimension();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
