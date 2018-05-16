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

//==============================================================================
template <class Handle>
template <typename Q, typename Enable>
ScopedState<Handle>::ScopedState(
    ScopedState<typename ScopedState<Handle>::NonConstHandle>&& other)
  : mBuffer(std::move(other.mBuffer))
{
  // Do nothing
}

//==============================================================================
template <class Handle>
template <typename Q, typename Enable>
typename ScopedState<Handle>::ScopedState& ScopedState<Handle>::operator=(
    ScopedState<typename ScopedState<Handle>::NonConstHandle>&& other)
{
  mBuffer = std::move(other.mBuffer);
}

//==============================================================================
template <class Handle>
ScopedState<Handle> ScopedState<Handle>::clone() const
{
  return this->mSpace->cloneState(this->mState);
}

} // namespace statespace
} // namespace aikido
