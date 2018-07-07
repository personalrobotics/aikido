#include "aikido/distance/ConfigurationRanker.hpp"

namespace {

struct sortByCost
{
  bool operator()(
      const std::pair<aikido::statespace::CartesianProduct::State*, double>&
          left,
      const std::pair<aikido::statespace::CartesianProduct::State*, double>&
          right)
  {
    return left.second < right.second;
  }
};
}

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
    std::vector<statespace::CartesianProduct::State*>& configurations)
{
  std::vector<std::pair<statespace::CartesianProduct::State*, double>>
      evaluatedConfigurations(configurations.size());
  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    evaluatedConfigurations[i].first = configurations[i];
    evaluatedConfigurations[i].second
        = evaluateConfiguration(configurations[i]);
  }

  std::sort(
      evaluatedConfigurations.begin(),
      evaluatedConfigurations.end(),
      sortByCost());

  configurations.clear();
  std::transform(
      evaluatedConfigurations.begin(),
      evaluatedConfigurations.end(),
      std::back_inserter(configurations),
      [](const std::pair<statespace::CartesianProduct::State*, double>& item) {
        return item.first;
      });
}

} // namespace distance
} // namespace aikido
