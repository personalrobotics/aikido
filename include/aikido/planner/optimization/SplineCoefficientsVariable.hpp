#ifndef AIKIDO_PLANNER_OPTIMIZATION_SPLINECOEFFICIENTSVARIABLE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_SPLINECOEFFICIENTSVARIABLE_HPP_

#include <memory>
#include "aikido/planner/optimization/SplineVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class SplineCoefficientsVariables : public SplineVariable
{
public:
  /// Constructor
  SplineCoefficientsVariables(
      const trajectory::Spline& splineToClone,
      bool fixedStartPoint = true,
      bool fixedEndPoint = true);

  // Documentation inherited.
  std::unique_ptr<Variable> clone() const override;

  // Documentation inherited.
  void setValue(const Eigen::VectorXd& variables) override;

  // Documentation inherited.
  Eigen::VectorXd getValue() const override;

  void freeStartPoint();

  void fixStartPoint();

  bool isStartPointFixed() const;

  void freeEndPoint();

  void fixEndPoint();

  bool isEndPointFixed() const;

protected:
  // Documentation inherited.
  void updateDimension() const override;

  bool mIsFixedStartPoint;

  bool mIsFixedEndPoint;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_SPLINECOEFFICIENTSVARIABLE_HPP_
