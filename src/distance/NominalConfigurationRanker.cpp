#include "aikido/distance/NominalConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::CartesianProduct;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
NominalConfigurationRanker::NominalConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : ConfigurationRanker(metaSkeletonStateSpace, metaSkeleton, ikSolutions)
{
  // Do nothing
}

//==============================================================================
double NominalConfigurationRanker::evaluateIKSolution(
    statespace::StateSpace::State* solution) const
{
  auto currentState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->convertPositionsToState(
      mMetaSkeleton->getPositions(), currentState);

  auto dmetric = createDistanceMetricFor(
      std::dynamic_pointer_cast<statespace::CartesianProduct>(
          std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
              mMetaSkeletonStateSpace)));
  return dmetric->distance(solution, currentState);
}

} // namespace distance
} // namespace aikido
