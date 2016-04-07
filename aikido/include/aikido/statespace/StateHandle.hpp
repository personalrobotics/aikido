#ifndef AIKIDO_STATESPACE_STATEHANDLE_H
#define AIKIDO_STATESPACE_STATEHANDLE_H
#include <cassert>

namespace aikido {
namespace statespace {

/// Wrap a State with its StateSpace to provide convenient accessor methods.
template <class _StateSpace, class _QualifiedState>
class StateHandle
{
public:
  using StateSpace = _StateSpace;
  using State = typename StateSpace::State;
  using QualifiedState = _QualifiedState;

  /// Constructs a nullptr handle.
  StateHandle()
    : mSpace(nullptr)
    , mState(nullptr)
  {
  }

  /// Wrap state, which must be form the provided StateSpace.
  StateHandle(const StateSpace* _space, QualifiedState* _state)
    : mSpace(_space)
    , mState(_state)
  {
  }

  StateHandle(const StateHandle&) = default;
  StateHandle(StateHandle&&) = default;

  StateHandle& operator =(StateHandle&&) = default;
  StateHandle& operator =(const StateHandle&) = default;

  /// Implicitly convert to a 
  operator QualifiedState*() const
  {
    return mState;
  }

  /// Reset StateHandle to nullptr.
  void reset()
  {
    mSpace = nullptr;
    mState = nullptr;
  }

  /// Reset the state, which must be from the provided StateSpace.
  void reset(const StateSpace* _space, QualifiedState* _state)
  {
    mSpace = _space;
    mState = _state;
  }

  /// Gets the State.
  QualifiedState* getState() const
  {
    return mState;
  }

  /// Gets the StateSpace.
  const StateSpace* getStateSpace() const
  {
    return mSpace;
  }

protected:
  const StateSpace* mSpace;
  QualifiedState* mState;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_STATEHANDLE_H
