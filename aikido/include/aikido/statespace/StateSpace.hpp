#ifndef AIKIDO_STATESPACE_STATESPACE_H
#define AIKIDO_STATESPACE_STATESPACE_H
#include <memory>
#include "ScopedState.hpp"

namespace aikido {
namespace statespace {

/// Base class for all StateSpaces.
class StateSpace {
public:
  /// Base class for all States.
  class State
  {
  protected:
    State() = default;

    /// It is unsafe to call this, since it is a non-virtual destructor.  Having
    /// any virtual function in this class, including this destructor, adds
    /// sizeof(pointer) overhead for a vtable.
    ~State() = default;
  };

  using StateHandle = statespace::StateHandle<StateSpace, State>;
  using StateHandleConst = statespace::StateHandle<StateSpace, const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  virtual ~StateSpace() = default;

  auto createState() const -> ScopedState
  {
    return ScopedState(this);
  }

  /// Gets the size of a State, in bytes.
  virtual size_t getStateSizeInBytes() const = 0;

  /// Allocate a new state. This must be deleted with freeState.
  virtual StateSpace::State* allocateState() const
  {
    return allocateStateInBuffer(new char[getStateSizeInBytes()]);
  }

  /// Create a new state in a pre-allocated buffer. The input argument must
  /// contain at least getStateSizeInBytes() bytes of memory.
  virtual StateSpace::State* allocateStateInBuffer(void* _buffer) const = 0;

  /// Free a state previously created by allocateState.
  virtual void freeState(StateSpace::State* _state) const
  {
    delete[] reinterpret_cast<char*>(_state);
  }

  /// Free a state previously created by allocateStateInBuffer.
  virtual void freeStateInBuffer(StateSpace::State* _state) const = 0;

  /// TODO: Need a docstring for this.
  virtual void compose(
    const State* _state1, const State* _state2, State* _out) const = 0;

  // TODO add '= 0' below to force all statespaces to implement
  /// Calculae the metric distance between two States
  virtual double distance(const State* state1, const State* state2) const;

  // TODO add '= 0' below to force all statespaces to implement
  /// Return a state alpha distance between this and other
  virtual void interpolate(const State* state1, const State* state2,
                           const double alpha, State* out) const;

  // TODO add '= 0' below to force all statespaces to implement
  /// Copy a State
  virtual void copyState(const State* state, State* stateCopy) const;
};

using StateSpacePtr = std::shared_ptr<StateSpace>;

} // namespace statespace
} // namespace aikido

#endif
