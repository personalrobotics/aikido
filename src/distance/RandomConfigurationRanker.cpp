#include <algorithm>

#include "aikido/distance/RandomConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
RandomConfigurationRanker::RandomConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : ConfigurationRanker(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // Do nothing
  //  std::random_shuffle(mIKSolutions.begin(), mIKSolutions.end());
}

//==============================================================================
double RandomConfigurationRanker::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  return 0;
  DART_UNUSED(solution);
}

} // namespace distance
} // namespace aikido
