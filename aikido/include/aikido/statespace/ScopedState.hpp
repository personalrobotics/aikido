#ifndef AIKIDO_STATESPACE_SCOPEDSTATE_H
#define AIKIDO_STATESPACE_SCOPEDSTATE_H
#include <memory>
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

template <class _StateSpace, class _QualifiedState>
class StateHandle
{
public:
  using StateSpace = _StateSpace;
  using State = typename StateSpace::State;
  using QualifiedState = _QualifiedState;

  StateHandle()
    : mSpace(nullptr)
    , mState(nullptr)
  {
  }

  StateHandle(const StateSpace* _space, QualifiedState* _state)
    : mSpace(_space)
    , mState(_state)
  {
  }

  StateHandle(const StateHandle&) = default;
  StateHandle(StateHandle&&) = default;

  StateHandle& operator =(StateHandle&&) = default;
  StateHandle& operator =(const StateHandle&) = default;

  operator QualifiedState*()
  {
    return mState;
  }

  QualifiedState* getState() const
  {
    return mState;
  }

  const StateSpace* getStateSpace() const
  {
    return mSpace;
  }

  QualifiedState& operator *()
  {
    return *mState;
  }

  QualifiedState* operator ->()
  {
    return mState;
  }

protected:
public:
  const StateSpace* mSpace;
  QualifiedState* mState;
};

template <class _Handle>
class ScopedState : public _Handle
{
public:
  using Handle = _Handle;
  using typename Handle::StateSpace;
  using typename Handle::State;
  using typename Handle::QualifiedState;

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
