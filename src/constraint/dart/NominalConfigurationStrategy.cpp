#include "aikido/constraint/dart/NominalConfigurationStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using distance::createDistanceMetricFor;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::CartesianProduct;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
NominalConfigurationStrategy::NominalConfigurationStrategy(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : IKRankingStrategy(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // Do nothing
}

//==============================================================================
double NominalConfigurationStrategy::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  auto currentState = mMetaSkeletonStateSpace->createState();
  auto currentPosition = mMetaSkeleton->getPositions();
  mMetaSkeletonStateSpace->convertPositionsToState(currentPosition, currentState);

  auto dmetric = createDistanceMetricFor(
        std::dynamic_pointer_cast<statespace::CartesianProduct>(
        std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(mMetaSkeletonStateSpace)));
  return dmetric->distance(solution, currentState);
}

} // namespace dart
} // namespace constraint
} // namespace aikido
