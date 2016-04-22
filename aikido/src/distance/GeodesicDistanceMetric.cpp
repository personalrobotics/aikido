#include <aikido/distance/GeodesicDistanceMetric.hpp>

namespace aikido {
namespace distance {

//=============================================================================
GeodesicDistanceMetric::GeodesicDistanceMetric(
    std::shared_ptr<statespace::SO3StateSpace> _space)
    : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("SO3StateSpace is nullptr.");
  }
}

//=============================================================================
statespace::StateSpacePtr GeodesicDistanceMetric::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
double GeodesicDistanceMetric::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  auto state1 = static_cast<const statespace::SO3StateSpace::State*>(_state1);
  auto state2 = static_cast<const statespace::SO3StateSpace::State*>(_state2);
  return mStateSpace->getQuaternion(state1)
      .angularDistance(mStateSpace->getQuaternion(state2));
}

}
}
