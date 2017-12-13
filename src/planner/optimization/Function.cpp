#include "aikido/planner/optimization/Function.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
double Function::evalFor(const Variable& variable)
{
  auto clone2 = clone();
  clone2->setVariable(variable);

  return clone2->eval(variable.getValue());
}

//==============================================================================
void Function::setVariable(const Variable& variableToClone)
{
  if (!isCompatible(variableToClone))
    throw std::invalid_argument("Invalid variable for this function.");

  mVariable = variableToClone.clone();
}

} // namespace optimization
} // namespace planner
} // namespace aikido
