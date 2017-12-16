#ifndef AIKIDO_PLANNER_OPTIMIZATION_COMPOSITIONFUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_COMPOSITIONFUNCTION_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/CompositeVariable.hpp"
#include "aikido/planner/optimization/Function.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class CompositeFunction : public Function
{
public:
  CompositeFunction(VariablePtr variable);

  virtual ~CompositeFunction();

  /// Clones this Function.
  UniqueFunctionPtr clone() const override;

  // Documentation inherited.
  bool isCompatible(const Variable& variable) const override;

  // Documentation inherited.
  double eval(const Eigen::VectorXd& x) override;

  void addSubFunction(FunctionPtr function, VariablePtr variable);

protected:
  CompositeVariablePtr getCompositeVariable();

  std::unordered_map<FunctionPtr, CompositeVariable::Index> mFunctionToVariable;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_COMPOSITIONFUNCTION_HPP_
