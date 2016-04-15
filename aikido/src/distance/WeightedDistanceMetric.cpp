#include <aikido/distance/WeightedDistanceMetric.hpp>

namespace aikido
{
namespace distance
{
WeightedDistanceMetric::WeightedDistanceMetric(
    std::shared_ptr<statespace::CompoundStateSpace> _space,
    std::vector<DistanceMetricPtr> _metrics)
    : mStateSpace(std::move(_space))
    , mMetrics(std::move(_metrics))
    , mWeights(mMetrics.size(), 1.0)
{
  if (mStateSpace->getNumStates() != mMetrics.size()) {
    throw std::invalid_argument(
        "Must provide a metric for every subspace in the CompoundStateSpace.");
  }
}

WeightedDistanceMetric::WeightedDistanceMetric(
    std::shared_ptr<statespace::CompoundStateSpace> _space,
    std::vector<DistanceMetricPtr> _metrics, std::vector<double> _weights)
    : mStateSpace(std::move(_space))
    , mMetrics(std::move(_metrics))
    , mWeights(std::move(_weights))
{
  if (mStateSpace->getNumStates() != mMetrics.size()) {
    throw std::invalid_argument(
        "Must provide a metric for every subspace in the CompoundStateSpace.");
  }

  if (mWeights.size() != mMetrics.size()) {
    throw std::invalid_argument("Must provide a weight for every metric.");
  }
}

statespace::StateSpacePtr WeightedDistanceMetric::getStateSpace() const
{
  return mStateSpace;
}

double WeightedDistanceMetric::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  auto state1 =
      static_cast<const statespace::CompoundStateSpace::State*>(_state1);
  auto state2 =
      static_cast<const statespace::CompoundStateSpace::State*>(_state2);

  double dist = 0.0;
  for (size_t i = 0; i < mMetrics.size(); ++i) {
    dist += mWeights[i]
            * mMetrics[i]->distance(mStateSpace->getSubState<>(state1, i),
                                    mStateSpace->getSubState<>(state2, i));
  }
  return dist;
}

}
}
