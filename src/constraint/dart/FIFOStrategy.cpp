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
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
  , mIndex(0.0)
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");

  mIKSolutions.resize(numIKSolutions);
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
