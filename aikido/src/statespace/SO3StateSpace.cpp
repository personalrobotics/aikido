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
SO3StateSpace::SO3StateSpace::State::State(const Quaternion& _quaternion)
  : mValue(_quaternion)
{
  // TODO: Check if normalized.
}

//=============================================================================
auto SO3StateSpace::State::getQuaternion() const -> const Quaternion&
{
  return mValue;
}

//=============================================================================
void SO3StateSpace::State::setQuaternion(const Quaternion& _quaternion)
{
  // TODO: Check if normalized.
  mValue = _quaternion;
}

//=============================================================================
auto SO3StateSpace::getQuaternion(const State* _state) const
  -> const Quaternion&
{
  return _state->mValue;
}

//=============================================================================
void SO3StateSpace::setQuaternion(
  State* _state, const Quaternion& _quaternion) const
{
  _state->mValue = _quaternion;
}

//=============================================================================
size_t SO3StateSpace::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* SO3StateSpace::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SO3StateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
void SO3StateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  out->mValue = state1->mValue * state2->mValue;
}

} // namespace statespace
} // namespace aikido
