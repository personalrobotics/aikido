#include <algorithm>
#include "aikido/constraint/dart/FIFOStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
FIFOStrategy::FIFOStrategy(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : IKRankingStrategy(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // Do nothing
//  std::random_shuffle(mIKSolutions.begin(), mIKSolutions.end());
}

//==============================================================================
double FIFOStrategy::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  return 0;
  DART_UNUSED(solution);
}

} // namespace dart
} // namespace constraint
} // namespace aikido
