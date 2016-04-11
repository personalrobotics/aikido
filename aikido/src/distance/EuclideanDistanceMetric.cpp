#include <aikido/distance/EuclideanDistanceMetric.hpp>

namespace aikido
{
namespace distance
{
EuclideanDistanceMetric::EuclideanDistanceMetric(
    std::shared_ptr<statespace::RealVectorStateSpace> _space)
    : mStateSpace(std::move(_space))
{
}

double EuclideanDistanceMetric::distance(
    const statespace::StateSpace::State* _state1,
    const statespace::StateSpace::State* _state2) const
{
  auto v1 = mStateSpace->getValue(
      static_cast<const statespace::RealVectorStateSpace::State*>(_state1));
  auto v2 = mStateSpace->getValue(
      static_cast<const statespace::RealVectorStateSpace::State*>(_state2));
  return (v2 - v1).norm();
}

void EuclideanDistanceMetric::interpolate(const statespace::StateSpace::State* _from,
                                    const statespace::StateSpace::State* _to,
                                    const double _t,
                                    statespace::StateSpace::State* _state) const
{
  if (_t < 0. || _t > 1.) {
    throw std::invalid_argument("_alpha must be between 0 and 1");
  }

  auto vfrom = mStateSpace->getValue(
      static_cast<const statespace::RealVectorStateSpace::State*>(_from));
  auto vto = mStateSpace->getValue(
      static_cast<const statespace::RealVectorStateSpace::State*>(_to));

  mStateSpace->setValue(static_cast<statespace::RealVectorStateSpace::State*>(_state),
                        (1 - _t) * vfrom + _t * vto);
}
}
}
