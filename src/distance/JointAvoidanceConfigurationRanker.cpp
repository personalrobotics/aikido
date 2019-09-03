#include "aikido/distance/JointAvoidanceConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using ::dart::dynamics::ConstMetaSkeletonPtr;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;

//==============================================================================
JointAvoidanceConfigurationRanker::JointAvoidanceConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    std::vector<double> weights)
  : ConfigurationRanker(
        std::move(metaSkeletonStateSpace), std::move(metaSkeleton), weights)
  , mLowerLimitsState(mMetaSkeletonStateSpace->createState())
  , mUpperLimitsState(mMetaSkeletonStateSpace->createState())
{
  setupJointLimits();
}

//==============================================================================
void JointAvoidanceConfigurationRanker::setupJointLimits()
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
    const statespace::dart::MetaSkeletonStateSpace::State* solution) const
{
  Eigen::VectorXd solutionPosition(mMetaSkeletonStateSpace->getDimension());
  mMetaSkeletonStateSpace->convertStateToPositions(solution, solutionPosition);

  auto lowerLimits = mMetaSkeleton->getPositionLowerLimits();
  auto upperLimits = mMetaSkeleton->getPositionUpperLimits();

  for (auto index : mUnboundedLowerLimitsIndices)
    lowerLimits[index] = solutionPosition[index];

  for (auto index : mUnboundedUpperLimitsIndices)
    upperLimits[index] = solutionPosition[index];

  mMetaSkeletonStateSpace->convertPositionsToState(
      lowerLimits, mLowerLimitsState);
  mMetaSkeletonStateSpace->convertPositionsToState(
      upperLimits, mUpperLimitsState);

  return -std::min(
      mDistanceMetric->distance(solution, mLowerLimitsState),
      mDistanceMetric->distance(solution, mUpperLimitsState));
}

} // namespace distance
} // namespace aikido
