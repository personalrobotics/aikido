#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace statespace {

//==============================================================================
auto StateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//==============================================================================
void StateSpace::compose(State* _state1, const State* _state2)
{
  auto tempState = createState();
  compose(_state1, _state2, tempState);
  copyState(tempState, _state1);
}

//==============================================================================
void StateSpace::getInverse(State* _state) const
{
  auto tempState = createState();
  getInverse(_state, tempState);
  copyState(tempState, _state);
}

//==============================================================================
auto StateSpace::allocateState() const -> State*
{
  return allocateStateInBuffer(new char[getStateSizeInBytes()]);
}

//==============================================================================
void StateSpace::freeState(StateSpace::State* _state) const
{
  delete[] reinterpret_cast<char*>(_state);
}

} // namespace statespace
} // namespace aikido
