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

  /// Get the dimension of the state space
  virtual unsigned int getDimension() const = 0;
  
  /// Get the maximum value a call to distance() can return (or an upper bound). 
  /// For unbounded state spaces, this function can return infinity.
  virtual double getMaximumExtent() const = 0;

  /// Get a measure of the space (this can be thought of as a generalization of volume) 
  virtual double getMeasure() const = 0;

  /// Check if a state is inside the bounding box. 
  /// For unbounded spaces this function can always return true.
  virtual bool satisfiesBounds(const StateSpace::State* _state) const = 0;

  /// Copy a state to another.
  virtual void copyState(StateSpace::State* _destination, 
                         const StateSpace::State* _source) const = 0;

  /// Computes distance between two states. This function satisfies 
  /// the properties of a metric if isMetricSpace() is true, and its 
  /// return value will always be between 0 and getMaximumExtent()
  virtual double distance(const StateSpace::State* _state1,
                          const StateSpace::State* _state2) const = 0;

  /// Equal states
  virtual bool equalStates(const StateSpace::State* _state1,
                           const StateSpace::State* _state2) const = 0;

  /// Computes the state that lies at time t in [0, 1] on the segment 
  /// that connects from state to to state. The memory location of state 
  /// is not required to be different from the memory of either from or to. 
  virtual void interpolate(const StateSpace::State* _from,
                           const StateSpace::State* _to,
                           const double _t,
                           StateSpace::State* _state) const = 0;
};

using StateSpacePtr = std::shared_ptr<StateSpace>;

} // namespace statespace
} // namespace aikido

#endif
