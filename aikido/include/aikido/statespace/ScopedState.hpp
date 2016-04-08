#ifndef AIKIDO_STATESPACE_SCOPEDSTATE_H
#define AIKIDO_STATESPACE_SCOPEDSTATE_H
#include <memory>
#include "StateHandle.hpp"

namespace aikido {
namespace statespace {

/// RAII wrapper for StateHandle.
template <class _Handle>
class ScopedState : public _Handle
{
public:
  using Handle = _Handle;
  using typename Handle::StateSpace;
  using typename Handle::State;
  using typename Handle::QualifiedState;

  /// Construct a ScopedState for a StateSpace.
  explicit ScopedState(const StateSpace* _space)
  {
    this->mSpace = _space;
    mBuffer.reset(new char[_space->getStateSizeInBytes()]);
    this->mState = static_cast<ScopedState::State*>(
      _space->allocateStateInBuffer(mBuffer.get()));
  }

  virtual ~ScopedState()
  {
    this->mSpace->freeStateInBuffer(this->mState);
  }

  // ScopedState is uncopyable, must use std::move
  ScopedState(const ScopedState&) = delete;
  ScopedState& operator =(const ScopedState&) = delete;

  ScopedState(ScopedState&&) = default;
  ScopedState& operator =(ScopedState&&) = default;

private:
  std::unique_ptr<char[]> mBuffer;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_SCOPEDSTATE_H
