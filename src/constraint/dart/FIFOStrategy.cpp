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
    std::size_t numIKSolutions)
  : IKRankingStrategy(metaSkeletonStateSpace, metaSkeleton, numIKSolutions)
{
  // Do nothing
}

//==============================================================================
double FIFOStrategy::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  DART_UNUSED(solution);
  return mIndex;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
