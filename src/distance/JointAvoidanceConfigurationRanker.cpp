#include "aikido/distance/JointAvoidanceConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
JointAvoidanceConfigurationRanker::JointAvoidanceConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : ConfigurationRanker(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // Do nothing
}

//==============================================================================
double JointAvoidanceConfigurationRanker::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  auto lowerLimitState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->convertPositionsToState(mMetaSkeleton->getPositionLowerLimits(), lowerLimitState);

  auto upperLimitState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->convertPositionsToState(mMetaSkeleton->getPositionUpperLimits(), upperLimitState);

  auto dmetric = createDistanceMetricFor(
        std::dynamic_pointer_cast<statespace::CartesianProduct>(
        std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(mMetaSkeletonStateSpace)));

  auto distanceFromLower = dmetric->distance(solution, lowerLimitState);
  auto distanceFromUpper = dmetric->distance(solution, upperLimitState);

  return -std::min(distanceFromLower, distanceFromUpper);
}

} // namespace distance
} // namespace aikido
