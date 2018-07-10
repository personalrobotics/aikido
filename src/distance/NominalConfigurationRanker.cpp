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
    const statespace::CartesianProduct::State* nominalConfiguration)
  : ConfigurationRanker(std::move(metaSkeletonStateSpace), std::move(metaSkeleton))
  , mNominalConfiguration(nominalConfiguration)
{
  if (!mNominalConfiguration)
    mNominalConfiguration
        = mMetaSkeletonStateSpace->getScopedStateFromMetaSkeleton(
            mMetaSkeleton.get());
}

//==============================================================================
double NominalConfigurationRanker::evaluateConfiguration(
    statespace::dart::MetaSkeletonStateSpace::State* solution) const
{
  return mDistanceMetric->distance(solution, mNominalConfiguration);
}

} // namespace distance
} // namespace aikido
