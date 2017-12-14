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
  /// Clones this Function.
  virtual std::shared_ptr<Function> clone() const = 0;
  // TODO(JS): Change this to unique_ptr

  /// Sets optimization variable associated with this Function.
  void setVariable(VariablePtr variable);
  // Dev-note: This function shouldn't be called other than by Optimizer.

  /// Returns optimization variable associated with this Function.
  ConstVariablePtr getVariable() const;

  /// Whether a variable is compatible this this function.
  virtual bool isCompatible(const Variable& variable) const = 0;

protected:
  /// Optimization variable.
  ///
  /// Variable can be nullptr in case this function hasn't been added to a
  /// Optimizer.
  VariablePtr mVariable;
};

using FunctionPtr = std::shared_ptr<Function>;
using ConstFunctionPtr = std::shared_ptr<const Function>;

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_FUNCTION_HPP_
