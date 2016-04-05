#ifndef AIKIDO_STATESPACE_SCOPEDSTATE_H
#define AIKIDO_STATESPACE_SCOPEDSTATE_H
#include <memory>
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

template <class _StateSpace>
class StateHandle
{
public:
  using StateSpace = _StateSpace;
  using State = typename StateSpace::State;

  StateHandle()
    : mSpace(nullptr)
    , mState(nullptr)
  {
  }

  StateHandle(const StateSpace* _space, State* _state)
    : mSpace(_space)
    , mState(_state)
  {
  }

  StateHandle(const StateHandle&) = default;
  StateHandle(StateHandle&&) = default;

  StateHandle& operator =(StateHandle&&) = default;
  StateHandle& operator =(const StateHandle&) = default;

  State* getState() const
  {
    return mState;
  }

  const StateSpace* getStateSpace() const
  {
    return mSpace;
  }

  State& operator *()
  {
    return *mState;
  }

  State* operator ->()
  {
    return mState;
  }

protected:
public:
  const StateSpace* mSpace;
  State* mState;
};

template <class _StateSpace>
class ScopedState : public StateHandle<_StateSpace>
{
public:
  using typename StateHandle<_StateSpace>::StateSpace;
  using typename StateHandle<_StateSpace>::State;

  explicit ScopedState(const StateSpace* _space)
    : StateHandle<StateSpace>()
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
