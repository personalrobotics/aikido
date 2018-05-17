#include "aikido/statespace/ScopedState.hpp"

namespace aikido {
namespace statespace {

//==============================================================================
template <class Handle>
ScopedState<Handle>::ScopedState(const StateSpace* _space)
{
  this->mSpace = _space;
  mBuffer.reset(new char[_space->getStateSizeInBytes()]);
  this->mState = static_cast<ScopedState::State*>(
      _space->allocateStateInBuffer(mBuffer.get()));
}

//==============================================================================
template <class Handle>
ScopedState<Handle>::~ScopedState()
{
  this->mSpace->freeStateInBuffer(this->mState);
}

} // namespace statespace
} // namespace aikido
