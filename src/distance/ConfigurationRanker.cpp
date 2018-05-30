#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
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
      std::dynamic_pointer_cast<statespace::CartesianProduct>(
          std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
              mMetaSkeletonStateSpace)));
}

//==============================================================================
statespace::ConstStateSpacePtr ConfigurationRanker::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
void ConfigurationRanker::rankConfigurations(
    std::vector<statespace::StateSpace::State*>& configurations)
{
  std::vector<std::pair<statespace::StateSpace::State*, double>>
      scoredConfigurations(configurations.size());
  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    scoredConfigurations[i].first = configurations[i];
    scoredConfigurations[i].second = evaluateConfiguration(configurations[i]);
  }

  struct sortingFunction
  {
    bool operator()(
        const std::pair<statespace::StateSpace::State*, double>& left,
        const std::pair<statespace::StateSpace::State*, double>& right)
    {
      return left.second < right.second;
    }
  };
  std::sort(
      scoredConfigurations.begin(),
      scoredConfigurations.end(),
      sortingFunction());

  configurations.clear();
  std::transform(
      scoredConfigurations.begin(),
      scoredConfigurations.end(),
      std::back_inserter(configurations),
      [](const std::pair<statespace::StateSpace::State*, int>& item) {
        return item.first;
      });
}

} // namespace distance
} // namespace aikido
