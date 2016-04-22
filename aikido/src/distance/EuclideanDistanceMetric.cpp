#include <aikido/distance/EuclideanDistanceMetric.hpp>

namespace aikido {
namespace distance {

//=============================================================================
EuclideanDistanceMetric::EuclideanDistanceMetric(
    std::shared_ptr<statespace::Rn> _space)
    : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("Rn is nullptr.");
  }
}

//=============================================================================
statespace::StateSpacePtr EuclideanDistanceMetric::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
double EuclideanDistanceMetric::distance(
    const statespace::StateSpace::State* _state1,
    const statespace::StateSpace::State* _state2) const
{
  auto v1 = mStateSpace->getValue(
      static_cast<const statespace::Rn::State*>(_state1));
  auto v2 = mStateSpace->getValue(
      static_cast<const statespace::Rn::State*>(_state2));
  return (v2 - v1).norm();
}

}
}
