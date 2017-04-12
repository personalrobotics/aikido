namespace aikido {
namespace statespace {

template <class _Handle>
ScopedState<_Handle>::ScopedState(const StateSpace* _space)
{
  this->mSpace = _space;
  mBuffer.reset(new char[_space->getStateSizeInBytes()]);
  this->mState = static_cast<ScopedState::State*>(
    _space->allocateStateInBuffer(mBuffer.get()));
}

template <class _Handle>
ScopedState<_Handle>::~ScopedState()
{
  this->mSpace->freeStateInBuffer(this->mState);
}

} // namespace statespace
} // namespace aikido
