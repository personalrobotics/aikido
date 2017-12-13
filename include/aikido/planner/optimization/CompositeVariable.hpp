#ifndef AIKIDO_PLANNER_OPTIMIZATION_COMPOSITEVARIABLE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_COMPOSITEVARIABLE_HPP_

#include <map>
#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/common/algorithm.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/optimization/Variable.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class CompositeVariable : public Variable
{
public:
  /// Clone
  std::unique_ptr<Variable> clone() const override;

  /// Returns the dimension of optimization variables.
  std::size_t getDimension() const override;

  /// Sets the optimization variables.
  void setValue(const Eigen::VectorXd& value) override;
  // TODO(JS): Change to setValues()

  /// Returns the optimization variables.
  Eigen::VectorXd getValue() const override;
  // TODO(JS): Change to getValues()

  void addVariable(const Variable& variableToClone);

protected:
  void updateDimension();

  std::vector<Variable> mVariables;

  std::size_t mDimension;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_COMPOSITEVARIABLE_HPP_
