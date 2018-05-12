#include "aikido/constraint/dart/JointAvoidanceStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
JointAvoidanceStrategy::JointAvoidanceStrategy(
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
double JointAvoidanceStrategy::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  DART_UNUSED(solution);
  // TODO (avk): Find the distance from the joint limits.
  // What if there are no joint limits like in SO(2) space?
  // Use distance metrics and not euclidean.
  return 0;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
