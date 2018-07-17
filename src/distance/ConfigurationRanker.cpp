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
  std::vector<std::pair<double, MetaSkeletonStateSpace::ScopedState>>
      evaluatedConfigurations;
  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    evaluatedConfigurations.emplace_back(
        std::make_pair(
            evaluateConfiguration(configurations[i]),
            std::move(configurations[i])));
  }
  std::sort(evaluatedConfigurations.begin(), evaluatedConfigurations.end());

  configurations.clear();
  for (std::size_t i = 0; i < evaluatedConfigurations.size(); ++i)
  {
    configurations.emplace_back(std::move(evaluatedConfigurations[i].second));
  }
}

} // namespace distance
} // namespace aikido
