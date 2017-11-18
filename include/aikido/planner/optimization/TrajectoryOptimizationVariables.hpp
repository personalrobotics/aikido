#ifndef AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYOPTIMIZATIONVARIABLES_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYOPTIMIZATIONVARIABLES_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class TrajectoryVariables
{
public:
  /// Clone
  virtual std::shared_ptr<TrajectoryVariables> clone() const = 0;

  /// Returns the dimension of optimization variables.
  virtual std::size_t getDimension() const = 0;

  /// Sets the optimization variables.
  virtual void setVariables(const Eigen::VectorXd& variables) = 0;

  /// Returns the optimization variables.
  virtual void getVariables(Eigen::VectorXd& variables) const = 0;

  /// Returns const Trajectory
  ///
  /// We only provide const version of this function because we don't intend
  /// the structure and parameters of spline to be changed by the caller.
  virtual const trajectory::Trajectory& getTrajectory() const = 0;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_TRAJECTORYOPTIMIZATIONVARIABLES_HPP_
