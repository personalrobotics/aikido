#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_

#include <memory>
#include "aikido/planner/optimization/SplineVariables.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineCoefficientsVariables : public SplineVariables
{
public:
  /// Constructor
  explicit SplineCoefficientsVariables(const trajectory::Spline& splineToClone);

  // Documentation inherited.
  std::shared_ptr<TrajectoryVariables> clone() const override;

  // Documentation inherited.
  void setVariables(const Eigen::VectorXd& variables) override;

  // Documentation inherited.
  void getVariables(Eigen::VectorXd& variables) const override;

protected:
  // Documentation inherited.
  void updateDimension() override;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
