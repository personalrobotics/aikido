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
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : IKRankingStrategy(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // Do nothing
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
