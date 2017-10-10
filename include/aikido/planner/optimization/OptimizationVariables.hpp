#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONTRAJECTORY_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONTRAJECTORY_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class OptimizationVariables
{
public:
  OptimizationVariables() = default;

  ~OptimizationVariables() = default;

  virtual std::shared_ptr<OptimizationVariables> clone() const = 0;

  virtual std::size_t getDimension() const = 0;

  virtual void setVariables(const Eigen::VectorXd& variables) = 0;

  virtual void getVariables(Eigen::VectorXd& variables) const = 0;

protected:
private:
};

class TrajectoryOptimizationVariables : virtual trajectory::Trajectory,
                                        public OptimizationVariables
{
public:
  TrajectoryOptimizationVariables() = default;

  ~TrajectoryOptimizationVariables() = default;

protected:
private:
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONTRAJECTORY_HPP_
