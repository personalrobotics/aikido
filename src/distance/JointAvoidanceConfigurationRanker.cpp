#include "aikido/distance/JointAvoidanceConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
JointAvoidanceConfigurationRanker::JointAvoidanceConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton)
  : ConfigurationRanker(metaSkeletonStateSpace, metaSkeleton)
{
  auto lowerLimits = mMetaSkeleton->getPositionLowerLimits();
  auto upperLimits = mMetaSkeleton->getPositionUpperLimits();

  for (std::size_t i = 0; i < mMetaSkeletonStateSpace->getDimension(); ++i)
  {
    if (lowerLimits[i] == -dart::math::constantsd::inf())
      mUnboundedLowerLimitsIndices.emplace_back(i);

    if (upperLimits[i] == dart::math::constantsd::inf())
      mUnboundedUpperLimitsIndices.emplace_back(i);
  }

  mMetaSkeletonStateSpace->convertPositionsToState(mMetaSkeleton->getPositionLowerLimits(), mLowerLimitsState);
  mMetaSkeletonStateSpace->convertPositionsToState(mMetaSkeleton->getPositionUpperLimits(), mUpperLimitsState);
}

//==============================================================================
double JointAvoidanceConfigurationRanker::evaluateConfiguration(
    statespace::CartesianProduct::State* solution) const
{
  Eigen::VectorXd solutionPosition(mMetaSkeletonStateSpace->getDimension());
  mMetaSkeletonStateSpace->convertStateToPositions(
      mMetaSkeletonStateSpace->cloneState(solution), solutionPosition);

  auto lowerLimits = mMetaSkeleton->getPositionLowerLimits();
  auto upperLimits = mMetaSkeleton->getPositionUpperLimits();

  for (auto index : mUnboundedLowerLimitsIndices)
    lowerLimits[index] = solutionPosition[index];

  for (auto index : mUnboundedUpperLimitsIndices)
    upperLimits[index] = solutionPosition[index];

  mMetaSkeletonStateSpace->convertPositionsToState(lowerLimits, mLowerLimitsState);
  mMetaSkeletonStateSpace->convertPositionsToState(upperLimits, mUpperLimitsState);

  return -std::min(
      mDistanceMetric->distance(solution, mLowerLimitsState),
      mDistanceMetric->distance(solution, mUpperLimitsState));
}

} // namespace distance
} // namespace aikido
