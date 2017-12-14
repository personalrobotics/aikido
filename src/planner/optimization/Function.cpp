#include "aikido/planner/optimization/Function.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
void Function::setVariable(VariablePtr variable)
{
  if (variable and !isCompatible(*variable))
    throw std::invalid_argument("Invalid variable for this function.");

  mVariable = std::move(variable);
}

//==============================================================================
ConstVariablePtr Function::getVariable() const
{
  return mVariable;
}

} // namespace optimization
} // namespace planner
} // namespace aikido
