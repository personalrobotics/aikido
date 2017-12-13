#include "aikido/planner/optimization/MetaSkeletonSplineCoefficientsAndDurationsVariable.hpp"

namespace aikido {
namespace planner {
namespace optimization {

//==============================================================================
MetaSkeletonSplineCoefficientsAndDurationsVariable::
    MetaSkeletonSplineCoefficientsAndDurationsVariable(
        const trajectory::Spline& splineToClone,
        dart::dynamics::MetaSkeletonPtr skeleton)
  : SplineCoefficientsAndDurationsVariable(splineToClone)
  , mSkeleton(std::move(skeleton))
{
  // TODO(JS): Throw if skeleton is nullptr
}

//==============================================================================
std::shared_ptr<Variable>
MetaSkeletonSplineCoefficientsAndDurationsVariable::clone() const
{
  return std::make_shared<MetaSkeletonSplineCoefficientsAndDurationsVariable>(
      *this);
}

//==============================================================================
void MetaSkeletonSplineCoefficientsAndDurationsVariable::
    setCoefficientValueAsJointPositionUpperLimitsTo(Eigen::VectorXd& vector)
{
  setCoefficientValueTo(vector, mSkeleton->getPositionUpperLimits());
}

//==============================================================================
void MetaSkeletonSplineCoefficientsAndDurationsVariable::
    setCoefficientValueAsJointPositionLowerLimitsTo(Eigen::VectorXd& vector)
{
  setCoefficientValueTo(vector, mSkeleton->getPositionLowerLimits());
}

//==============================================================================
void MetaSkeletonSplineCoefficientsAndDurationsVariable::
    setCoefficientValueAsJointMidPointsOfLimitsTo(Eigen::VectorXd& vector)
{
  setCoefficientValueTo(
      vector,
      0.5 * (mSkeleton->getPositionLowerLimits()
             + mSkeleton->getPositionUpperLimits()));
}

//==============================================================================
void MetaSkeletonSplineCoefficientsAndDurationsVariable::
    setCoefficientValueAsCurrentJointPositionsTo(Eigen::VectorXd& vector)
{
  setCoefficientValueTo(vector, mSkeleton->getPositions());
}

} // namespace optimization
} // namespace planner
} // namespace aikido
