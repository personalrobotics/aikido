#include "aikido/constraint/dart/NominalConfigurationStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
NominalConfigurationStrategy::NominalConfigurationStrategy(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const std::vector<statespace::StateSpace::ScopedState> ikSolutions)
  : IKRankingStrategy(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // Do nothing
}

//==============================================================================
double NominalConfigurationStrategy::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  DART_UNUSED(solution);
  // TODO (avk): Find the distance from the current state.
  // Use distance metrics and not euclidean.
  return 0;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
