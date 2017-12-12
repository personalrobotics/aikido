#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_

#include <memory>
#include "aikido/planner/optimization/SplineVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineCoefficientsAndDurationsVariables : public SplineVariables
{
public:
  /// Constructor
  SplineCoefficientsAndDurationsVariables(
      const trajectory::Spline& splineToClone);

  // Documentation inherited.
  std::shared_ptr<Variable> clone() const override;

  // Documentation inherited.
  void setValue(const Eigen::VectorXd& value) override;

  // Documentation inherited.
  Eigen::VectorXd getValue() const override;

  void setCoefficientValueTo(Eigen::VectorXd& vector, double value) const;

  void setCoefficientValueTo(
      Eigen::VectorXd& vector, const Eigen::VectorXd& values) const;

  void setDurationValueTo(
      Eigen::VectorXd& vector, const Eigen::VectorXd& duration);

  void setDurationValueTo(
      Eigen::VectorXd& vector, std::size_t segmentIndex, double duration);

protected:
  // Documentation inherited.
  void updateDimension() override;
};

void setCoefficientValueAsJointPositionUpperLimitsTo(
    Eigen::VectorXd& vector,
    const SplineCoefficientsAndDurationsVariables& variables,
    const dart::dynamics::MetaSkeleton& skeleton);

void setCoefficientValueAsJointPositionLowerLimitsTo(
    Eigen::VectorXd& vector,
    const SplineCoefficientsAndDurationsVariables& variables,
    const dart::dynamics::MetaSkeleton& skeleton);

void setCoefficientValueAsJointMidPointsOfLimitsTo(
    Eigen::VectorXd& vector,
    const SplineCoefficientsAndDurationsVariables& variables,
    const dart::dynamics::MetaSkeleton& skeleton);

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
