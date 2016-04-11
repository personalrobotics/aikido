#include <aikido/distance/GeodesicDistanceMetric.hpp>

namespace aikido
{
namespace distance
{
GeodesicDistanceMetric::GeodesicDistanceMetric(
    std::shared_ptr<statespace::SO3StateSpace> _space)
    : mStateSpace(_space)
{
}

double GeodesicDistanceMetric::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  auto state1 = static_cast<const statespace::SO3StateSpace::State*>(_state1);
  auto state2 = static_cast<const statespace::SO3StateSpace::State*>(_state2);
  return mStateSpace->getQuaternion(state1)
      .angularDistance(mStateSpace->getQuaternion(state2));
}

void GeodesicDistanceMetric::interpolate(
    const aikido::statespace::StateSpace::State* _from,
    const aikido::statespace::StateSpace::State* _to, const double _t,
    aikido::statespace::StateSpace::State* _state) const
{
  auto from = static_cast<const statespace::SO3StateSpace::State*>(_from);
  auto to = static_cast<const statespace::SO3StateSpace::State*>(_to);
  auto state = static_cast<statespace::SO3StateSpace::State*>(_state);

  mStateSpace->setQuaternion(state, mStateSpace->getQuaternion(from).slerp(
                                        _t, mStateSpace->getQuaternion(to)));
}
}
}
