#ifndef AIKIDO_STATESPACE_STATESPACE_H
#define AIKIDO_STATESPACE_STATESPACE_H
#include <memory>

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

  using StateUniquePtr = std::unique_ptr<
    StateSpace::State, std::function<void (StateSpace::State*)>>;

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

  /// Helper function that wraps allocate/freeState in a unique_ptr. Note that
  /// this unique_ptr introduces sizeof(pointer) overhead because it uses a
  /// custom deleter.
  StateUniquePtr allocateManagedState() const
  {
    return StateUniquePtr(allocateState(),
      [this] (StateSpace::State* _state) -> void
      {
        this->freeState(_state);
      }
    );
  }

  /// TODO: Need a docstring for this.
  virtual void compose(const State& _state1, const State& _state2,
                       State& _out) const = 0;
};

using StateSpacePtr = std::shared_ptr<StateSpace>;

} // namespace statespace
} // namespace aikido

#endif
