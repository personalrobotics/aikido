#include <aikido/statespace/StateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
auto StateSpace::allocateState() const -> State*
{
  return allocateStateInBuffer(new char[getStateSizeInBytes()]);
}

//=============================================================================
void StateSpace::freeState(StateSpace::State* _state) const
{
  delete[] reinterpret_cast<char*>(_state);
}

} // namespace statespace
} // namespace aikido
