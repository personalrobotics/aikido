#include <aikido/statespace/SO3StateSpace.hpp>
#include <dart/math/Geometry.h>
#include <iostream>

namespace aikido {
namespace statespace {

//=============================================================================
SO3StateSpace::SO3StateSpace::State::State()
  : mValue(1., 0., 0., 0.)
{
}

//=============================================================================
SO3StateSpace::SO3StateSpace::State::State(
      const Eigen::Quaterniond& _quaternion)
  : mValue(_quaternion)
{
  // TODO: Check if normalized.
}

//=============================================================================
const Eigen::Quaterniond& SO3StateSpace::State::getQuaternion() const
{
  return mValue;
}

//=============================================================================
void SO3StateSpace::State::setQuaternion(const Eigen::Quaterniond& _quaternion)
{
  // TODO: Check if normalized.
  mValue = _quaternion;
}

//=============================================================================
int SO3StateSpace::getRepresentationDimension() const
{
  return 3;
}

//=============================================================================
void SO3StateSpace::compose(
  const StateSpace::State& _state1, const StateSpace::State& _state2,
  StateSpace::State& _out) const
{
  const auto& state1 = static_cast<const State&>(_state1);
  const auto& state2 = static_cast<const State&>(_state2);
  auto& out = static_cast<State&>(_out);

  out.mValue = state1.mValue * state2.mValue;
}

} // namespace statespace
} // namespace aikido
