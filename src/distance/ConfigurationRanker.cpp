#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSpace;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
ConfigurationRanker::ConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton)
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");

  mDistanceMetric = createDistanceMetricFor(
      std::dynamic_pointer_cast<const statespace::CartesianProduct>(
          mMetaSkeletonStateSpace));
}

//==============================================================================
void ConfigurationRanker::rankConfigurations(
    std::vector<MetaSkeletonStateSpace::ScopedState>& configurations)
{
  std::sort(
      configurations.begin(),
      configurations.end(),
      [&](const MetaSkeletonStateSpace::ScopedState& left,
          const MetaSkeletonStateSpace::ScopedState& right) {
        return evaluateConfiguration(left) < evaluateConfiguration(right);
      });
}

} // namespace distance
} // namespace aikido
