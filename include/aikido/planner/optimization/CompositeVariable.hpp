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
  using Index = std::size_t;
  static Index InvalidIndex;

  /// Clones this CompositeVariable.
  UniqueVariablePtr clone() const override;

  /// Returns the dimension of optimization variables.
  std::size_t getDimension() const override;

  /// Sets the optimization variables.
  void setValue(const Eigen::VectorXd& value) override;

  /// Returns the optimization variables.
  Eigen::VectorXd getValue() const override;

  Eigen::Map<const Eigen::VectorXd> getSubValue(
      const Eigen::VectorXd& value, const Variable* variable) const;

  std::size_t addSubVariable(VariablePtr variable);

  /// Returns a sub variable by an index
  ConstVariablePtr getSubVariable(std::size_t index) const;

  /// Returns an index of a sub variable
  std::size_t getSubVariableIndex(const Variable* variable) const;

  /// Returns whether this CompositeVariable contains the Variable.
  bool hasSubVariable(const Variable* variable) const;

protected:
  /// Updates the dimension of this CompositeVariable.
  ///
  /// This function should be called when sub variables are added or removed.
  void updateDimension();

  std::vector<VariablePtr> mSubVariables;

  std::size_t mDimension;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_COMPOSITEVARIABLE_HPP_
