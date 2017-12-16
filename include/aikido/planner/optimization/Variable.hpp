#ifndef AIKIDO_PLANNER_OPTIMIZATION_VARIABLE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_VARIABLE_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/common/algorithm.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/optimization/smart_pointer.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class TrajectoryVariable;

class Variable
{
public:
  /// Clone
  virtual UniqueVariablePtr clone() const = 0;

  /// Returns the dimension of optimization variables.
  virtual std::size_t getDimension() const = 0;

  /// Sets the optimization variables.
  virtual void setValue(const Eigen::VectorXd& value) = 0;

  /// Returns the optimization variables.
  virtual Eigen::VectorXd getValue() const = 0;

  Eigen::VectorXd createValue() const;
  Eigen::VectorXd createValue(double value) const;
  Eigen::VectorXd createZeroValue() const;

  virtual void generateRandomValueTo(
      Eigen::VectorXd& value,
      const Eigen::VectorXd& min,
      const Eigen::VectorXd& max) const
  {
    value.resize(getDimension());
    value.setRandom();

    for (auto i = 0; i < value.size(); ++i)
      value[i] = common::clamp(value[i], min[i], max[i]);
  }
  // TODO(JS): Make this as a pure function

  virtual bool isTrajectoryVariable() const;

  virtual TrajectoryVariable* asTrajectoryVariable();

  virtual const TrajectoryVariable* asTrajectoryVariable() const;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_VARIABLE_HPP_
