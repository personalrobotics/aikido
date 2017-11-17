#ifndef AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_

#include <memory>

#include <dart/optimizer/optimizer.hpp>

#include "aikido/planner/optimization/TrajectoryOptimizationVariables.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineVariables : public TrajectoryOptimizationVariables
{
public:
  SplineVariables(const trajectory::Spline& splineToClone);

  // Documentation inherited.
  std::shared_ptr<TrajectoryOptimizationVariables> clone() const override;

  // Documentation inherited.
  std::size_t getDimension() const override;

  // Documentation inherited.
  void setVariables(const Eigen::VectorXd& variables) override;

  // Documentation inherited.
  void getVariables(Eigen::VectorXd& variables) const override;

  /// Returns Spline trajectory
  const trajectory::Spline* getSpline() const;

protected:
  /// Returns Spline trajectory
  trajectory::Spline* getSpline();
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_SPLINEVARIABLES_HPP_
