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
}

//==============================================================================
double JointAvoidanceConfigurationRanker::evaluateConfiguration(
    statespace::StateSpace::State* solution) const
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

  auto lowerLimitsState
      = mMetaSkeletonStateSpace->getStateFromPositions(lowerLimits);
  auto upperLimitsState
      = mMetaSkeletonStateSpace->getStateFromPositions(upperLimits);

  return -std::min(
      mDistanceMetric->distance(solution, lowerLimitsState),
      mDistanceMetric->distance(solution, upperLimitsState));
}

} // namespace distance
} // namespace aikido
