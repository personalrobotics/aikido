#include <dart/common/StlHelpers.hpp>

#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSpace;
using dart::common::make_unique;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
ConfigurationRanker::ConfigurationRanker(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    std::vector<double> weights)
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");

  if (weights.size() != 0)
  {
    if (weights.size() != mMetaSkeletonStateSpace->getDimension())
      throw std::invalid_argument(
          "Weights size should match MetaSkeletonStateSpace dimension.");

    for (std::size_t i = 0; i < weights.size(); ++i)
    {
      if (weights[i] < 0)
        throw std::invalid_argument("Weights should all be non-negative.");
    }
  }
  else
  {
    weights.resize(mMetaSkeletonStateSpace->getDimension());
    for (std::size_t i = 0; i < weights.size(); ++i)
    {
      weights[i] = 1;
    }
  }

  // Create a temporary statespace to setup distance metric with weights.
  auto _sspace = std::dynamic_pointer_cast<statespace::CartesianProduct>(
      std::const_pointer_cast<MetaSkeletonStateSpace>(mMetaSkeletonStateSpace));

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
void ConfigurationRanker::rankConfigurations(
    std::vector<MetaSkeletonStateSpace::ScopedState>& configurations)
{
  std::unordered_map<const MetaSkeletonStateSpace::State*, double> costs;
  costs.reserve(configurations.size());
  for (std::size_t i = 0; i < configurations.size(); ++i)
    costs[configurations[i]] = evaluateConfiguration(configurations[i]);

  std::sort(
      configurations.begin(),
      configurations.end(),
      [&](const MetaSkeletonStateSpace::ScopedState& left,
          const MetaSkeletonStateSpace::ScopedState& right) {
        return costs[left] < costs[right];
      });
}

} // namespace distance
} // namespace aikido
