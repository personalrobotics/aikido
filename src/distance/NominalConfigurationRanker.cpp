#include "aikido/distance/NominalConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using ::dart::dynamics::ConstMetaSkeletonPtr;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;

//==============================================================================
NominalConfigurationRanker::NominalConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    const statespace::CartesianProduct::State* nominalConfiguration,
    std::vector<double> weights)
  : ConfigurationRanker(
        std::move(metaSkeletonStateSpace), std::move(metaSkeleton), weights)
  , mNominalConfiguration(
        mMetaSkeletonStateSpace->cloneState(nominalConfiguration))
{
  // Do nothing
}

//==============================================================================
NominalConfigurationRanker::NominalConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    std::vector<double> weights)
  : ConfigurationRanker(
        std::move(metaSkeletonStateSpace), std::move(metaSkeleton), weights)
  , mNominalConfiguration(
        mMetaSkeletonStateSpace->getScopedStateFromMetaSkeleton(
            mMetaSkeleton.get()))
{
  // Do nothing
}

//==============================================================================
double NominalConfigurationRanker::evaluateConfiguration(
    const statespace::dart::MetaSkeletonStateSpace::State* solution) const
{
  return mDistanceMetric->distance(solution, mNominalConfiguration);
}

} // namespace distance
} // namespace aikido
