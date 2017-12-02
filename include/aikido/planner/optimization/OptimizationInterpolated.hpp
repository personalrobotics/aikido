#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONINTERPOLATED_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONINTERPOLATED_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/TrajectoryVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class OptimizationInterpolated : public TrajectoryVariable
{
public:
  OptimizationInterpolated() = default;

  ~OptimizationInterpolated() = default;

  virtual void setValue(const Eigen::VectorXd& variables) = 0;

  virtual Eigen::VectorXd getValue() const = 0;

protected:
private:
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONINTERPOLATED_HPP_
