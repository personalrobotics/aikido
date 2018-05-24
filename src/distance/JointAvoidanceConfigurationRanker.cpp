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
  mPositionLowerLimits = mMetaSkeleton->getPositionLowerLimits();
  mPositionUpperLimits = mMetaSkeleton->getPositionUpperLimits();
}

//==============================================================================
double JointAvoidanceConfigurationRanker::evaluateConfiguration(
    statespace::StateSpace::State* solution) const
{
  Eigen::VectorXd solutionPosition(mMetaSkeletonStateSpace->getDimension());
  mMetaSkeletonStateSpace->convertStateToPositions(
      mMetaSkeletonStateSpace->cloneState(solution), solutionPosition);

  auto lowerLimitPosition = mPositionLowerLimits;
  auto upperLimitPosition = mPositionUpperLimits;

  // TODO (avk): Get the indices once?
  for (std::size_t i = 0; i < mMetaSkeletonStateSpace->getDimension(); ++i)
  {
    if (lowerLimitPosition[i] == -dart::math::constantsd::inf())
      lowerLimitPosition[i] = solutionPosition[i];

    if (upperLimitPosition[i] == dart::math::constantsd::inf())
      upperLimitPosition[i] = solutionPosition[i];
  }

  auto lowerLimitState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->convertPositionsToState(
      lowerLimitPosition, lowerLimitState);

  auto upperLimitState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->convertPositionsToState(
      lowerLimitPosition, upperLimitState);

  auto distanceFromLower = mDistanceMetric->distance(solution, lowerLimitState);
  auto distanceFromUpper = mDistanceMetric->distance(solution, upperLimitState);

  return -std::min(distanceFromLower, distanceFromUpper);
}

} // namespace distance
} // namespace aikido
