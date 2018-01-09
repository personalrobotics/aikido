#ifndef AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_

#include <memory>
#include "aikido/planner/optimization/SplineVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineCoefficientsAndDurationsVariable : public SplineVariable
{
public:
  /// Constructor
  SplineCoefficientsAndDurationsVariable(
      const trajectory::Spline& splineToClone,
      bool fixedStartPoint = true,
      bool fixedEndPoint = true);

  // Documentation inherited.
  std::unique_ptr<Variable> clone() const override;

  // Documentation inherited.
  void setValue(const Eigen::VectorXd& value) override;

  // Documentation inherited.
  Eigen::VectorXd getValue() const override;

  void freeStartPoint();

  void fixStartPoint();

  bool isStartPointFixed() const;

  void freeEndPoint();

  void fixEndPoint();

  bool isEndPointFixed() const;

  void setCoefficientValueTo(Eigen::VectorXd& vector, double value) const;

  void setCoefficientValueTo(
      Eigen::VectorXd& vector, const Eigen::VectorXd& values) const;

  void setDurationValueTo(Eigen::VectorXd& vector, double duration);

  void setDurationValueTo(
      Eigen::VectorXd& vector, const Eigen::VectorXd& duration);

  void setDurationValueTo(
      Eigen::VectorXd& vector, std::size_t segmentIndex, double duration);

protected:
  // Documentation inherited.
  void updateDimension() const override;

  bool mIsFixedStartPoint;

  bool mIsFixedEndPoint;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_OPTIMIZATIONSPLINE_HPP_
