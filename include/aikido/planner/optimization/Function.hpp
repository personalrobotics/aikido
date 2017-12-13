#ifndef AIKIDO_PLANNER_OPTIMIZATION_FUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_FUNCTION_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class Function : public dart::optimizer::Function
{
public:
  Function() = default;

  virtual ~Function() = default;

  virtual std::shared_ptr<Function> clone() const = 0;
  // TODO(JS): Change this to unique_ptr

  double evalFor(const Variable& variable);
  // Helper function to compute the value of this function for given Variable
  // instead of using the internal Variable.

protected:
  friend class Optimizer;

  void setVariable(const Variable& variableToClone);
  // Dev-note: This function shouldn't be called other than by Optimizer.

  virtual bool isCompatible(const Variable& variable) const = 0;

  std::shared_ptr<Variable> mVariable;
};

using FunctionPtr = std::shared_ptr<Function>;
using ConstFunctionPtr = std::shared_ptr<const Function>;

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_FUNCTION_HPP_
