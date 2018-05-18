#include <algorithm>
#include "aikido/constraint/dart/FIFOStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

// TODO(avk) This is only to test the API. I think this class can be deleted.
// Only downside is that we might always have the IK with current state as seed
// as the first element in the vector even if we want random.

//==============================================================================
FIFOStrategy::FIFOStrategy(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : IKRankingStrategy(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // TODO (avk): how to write test if the following is added?
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
