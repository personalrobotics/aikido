#include <dart/common/StlHelpers.hpp>

#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;
using dart::common::make_unique;

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

  // mDistanceMetric = createDistanceMetricFor(
  //     std::dynamic_pointer_cast<statespace::CartesianProduct>(
  //         std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
  //             mMetaSkeletonStateSpace)));

  std::vector<double> weights{1,1,1,1,1,1};

  auto _sspace = std::dynamic_pointer_cast<statespace::CartesianProduct>(
          std::const_pointer_cast<statespace::dart::MetaSkeletonStateSpace>(
              mMetaSkeletonStateSpace));

  std::vector<std::pair<DistanceMetricPtr, double>> metrics;
  metrics.reserve(_sspace->getNumSubspaces());

  for (std::size_t i = 0; i < _sspace->getNumSubspaces(); ++i)
  {
    auto subspace = _sspace->getSubspace<>(i);
    auto metric = createDistanceMetric(std::move(subspace));
    metrics.emplace_back(std::make_pair(std::move(metric), weights[i]));
  }

  mDistanceMetric = make_unique<CartesianProductWeighted>(
      std::move(_sspace), std::move(metrics));

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
      scoredConfigurations(configurations.size());
  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    scoredConfigurations[i].first = configurations[i];
    scoredConfigurations[i].second = evaluateConfiguration(configurations[i]);
  }

  struct sortingFunction
  {
    bool operator()(
        const std::pair<statespace::CartesianProduct::State*, double>& left,
        const std::pair<statespace::CartesianProduct::State*, double>& right)
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
      [](const std::pair<statespace::CartesianProduct::State*, int>& item) {
        return item.first;
      });
}

} // namespace distance
} // namespace aikido
