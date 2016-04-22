#include <aikido/distance/WeightedDistanceMetric.hpp>

namespace aikido {
namespace distance {

//=============================================================================
WeightedDistanceMetric::WeightedDistanceMetric(
    std::shared_ptr<statespace::CartesianProduct> _space,
    std::vector<DistanceMetricPtr> _metrics)
    : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("CartesianProduct is nullptr");
  }

  if (mStateSpace->getNumStates() != _metrics.size()) {
    std::stringstream msg;
    msg << "Must provide a metric for every subspace in the "
           "CartesianProduct. "
        << " (subspaces = " << mStateSpace->getNumStates()
        << " , metrics = " << _metrics.size() << ")";
    throw std::invalid_argument(msg.str());
  }

  mMetrics.reserve(_metrics.size());
  for (size_t i = 0; i < mStateSpace->getNumStates(); ++i) {
    if (_metrics[i] == nullptr) {
      std::stringstream msg;
      msg << "DistanceMetric " << i << " is nullptr.";
      throw std::invalid_argument(msg.str());
    }

    if (mStateSpace->getSubSpace<>(i) != _metrics[i]->getStateSpace()) {
      std::stringstream msg;
      msg << "DistanceMetric " << i
          << " is not defined over the correct StateSpace.";
      throw std::invalid_argument(msg.str());
    }
    mMetrics.emplace_back(std::move(_metrics[i]), 1);
  }
}

//=============================================================================
WeightedDistanceMetric::WeightedDistanceMetric(
    std::shared_ptr<statespace::CartesianProduct> _space,
    std::vector<std::pair<DistanceMetricPtr, double>> _metrics)
    : mStateSpace(std::move(_space))
    , mMetrics(std::move(_metrics))
{
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("CartesianProduct is nullptr");
  }

  if (mStateSpace->getNumStates() != mMetrics.size()) {
    std::stringstream msg;
    msg << "Must provide a metric for every subspace in the "
           "CartesianProduct. "
        << " (subspaces = " << mStateSpace->getNumStates()
        << " , metrics = " << mMetrics.size() << ")";
    throw std::invalid_argument(msg.str());
  }

  for (size_t i = 0; i < mStateSpace->getNumStates(); ++i) {
    if (mMetrics[i].first == nullptr) {
      std::stringstream msg;
      msg << "DistanceMetric " << i << " is nullptr.";
      throw std::invalid_argument(msg.str());
    }
    if (mStateSpace->getSubSpace<>(i) != mMetrics[i].first->getStateSpace()) {
      std::stringstream msg;
      msg << "DistanceMetric " << i
          << " is not defined over the correct StateSpace.";
      throw std::invalid_argument(msg.str());
    }
    if (mMetrics[i].second < 0) {
      std::stringstream msg;
      msg << "The weight for subspace " << i
          << " is negative. All weights must be positive.";
      throw std::invalid_argument(msg.str());
    }
  }
}

//=============================================================================
statespace::StateSpacePtr WeightedDistanceMetric::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
double WeightedDistanceMetric::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  auto state1 =
      static_cast<const statespace::CartesianProduct::State*>(_state1);
  auto state2 =
      static_cast<const statespace::CartesianProduct::State*>(_state2);

  double dist = 0.0;
  for (size_t i = 0; i < mMetrics.size(); ++i) {
    dist +=
        mMetrics[i].second
        * mMetrics[i].first->distance(mStateSpace->getSubState<>(state1, i),
                                      mStateSpace->getSubState<>(state2, i));
  }
  return dist;
}
}
}
