#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_

#include <memory>
#include "aikido/planner/optimization/SplineVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineCoefficientsVariables : public SplineVariables
{
public:
  /// Constructor
  explicit SplineCoefficientsVariables(const trajectory::Spline& splineToClone);

  // Documentation inherited.
  std::shared_ptr<Variable> clone() const override;

  // Documentation inherited.
  void setValue(const Eigen::VectorXd& variables) override;

  // Documentation inherited.
  Eigen::VectorXd getValue() const override;

protected:
  // Documentation inherited.
  void updateDimension() override;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
