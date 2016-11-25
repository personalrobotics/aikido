#include <aikido/distance/SO3Angular.hpp>

namespace aikido {
namespace distance {

//=============================================================================
SO3Angular::SO3Angular(
    std::shared_ptr<statespace::SO3> _space)
    : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("SO3 is nullptr.");
  }
}

//=============================================================================
statespace::StateSpacePtr SO3Angular::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
double SO3Angular::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  auto state1 = static_cast<const statespace::SO3::State*>(_state1);
  auto state2 = static_cast<const statespace::SO3::State*>(_state2);
  return mStateSpace->getQuaternion(state1)
      .angularDistance(mStateSpace->getQuaternion(state2));
}

}
}
