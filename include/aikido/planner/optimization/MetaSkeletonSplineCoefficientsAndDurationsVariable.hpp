#ifndef AIKIDO_PLANNER_OPTIMIZATION_METASKELETONSPLINECOEFFICIENTSANDDURATIONSVARIABLE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_METASKELETONSPLINECOEFFICIENTSANDDURATIONSVARIABLE_HPP_

#include <memory>
#include "aikido/planner/optimization/SplineCoefficientsAndDurationsVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

class MetaSkeletonSplineCoefficientsAndDurationsVariable
    : public SplineCoefficientsAndDurationsVariable
{
public:
  /// Constructor
  MetaSkeletonSplineCoefficientsAndDurationsVariable(
      const trajectory::Spline& splineToClone,
      dart::dynamics::MetaSkeletonPtr skeleton);

  // Documentation inherited.
  std::shared_ptr<Variable> clone() const override;

  void setCoefficientValueAsJointPositionUpperLimitsTo(Eigen::VectorXd& vector);

  void setCoefficientValueAsJointPositionLowerLimitsTo(Eigen::VectorXd& vector);

  void setCoefficientValueAsJointMidPointsOfLimitsTo(Eigen::VectorXd& vector);

  void setCoefficientValueAsCurrentJointPositionsTo(Eigen::VectorXd& vector);

protected:
  dart::dynamics::MetaSkeletonPtr mSkeleton;
};

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_METASKELETONSPLINECOEFFICIENTSANDDURATIONSVARIABLE_HPP_
