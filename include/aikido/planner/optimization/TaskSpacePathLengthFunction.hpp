#ifndef AIKIDO_PLANNER_OPTIMIZATION_TASKSPACEPATHLENGTHFUNCTION_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_TASKSPACEPATHLENGTHFUNCTION_HPP_

#include <dart/optimizer/Function.hpp>
#include "aikido/planner/optimization/TrajectoryFunction.hpp"
#include "aikido/planner/optimization/TrajectoryVariable.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

// TODO(JS): Not implemented
class TaskSpacePathLengthFunction : public TrajectoryFunction
{
public:
  /// Constructor
  explicit TaskSpacePathLengthFunction(TrajectoryVariablePtr variable);

  // Documentation inherited.
  UniqueFunctionPtr clone() const override;

  // Documentation inherited.
  double eval(const Eigen::VectorXd& x) override;

  // TODO(JS): Add setter/getter for distance metric

protected:
  distance::DistanceMetricPtr mDistanceMetric;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_TASKSPACEPATHLENGTHFUNCTION_HPP_
