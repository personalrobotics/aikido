#ifndef AIKIDO_STATESPACE_STATESPACE_H
#define AIKIDO_STATESPACE_STATESPACE_H
#include <memory>
#include "ScopedState.hpp"
#include <Eigen/Dense>
#include "../util/RNG.hpp"

namespace aikido
{
namespace statespace
{
/// Base class for all StateSpaces.
class StateSpace
{
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

  auto createState() const -> ScopedState { return ScopedState(this); }

  /// Allocate a new state. This must be deleted with freeState.
  virtual State *allocateState() const;

  /// Free a state previously created by allocateState.
  virtual void freeState(State *_state) const;

  /// Gets the size of a State, in bytes.
  virtual size_t getStateSizeInBytes() const = 0;

  /// Create a new state in a pre-allocated buffer. The input argument must
  /// contain at least getStateSizeInBytes() bytes of memory.
  virtual State *allocateStateInBuffer(void *_buffer) const = 0;

  /// Free a state previously created by allocateStateInBuffer.
  virtual void freeStateInBuffer(State *_state) const = 0;

  /// Lie group operation for this StateSpace.
  virtual void compose(const State *_state1, const State *_state2,
                       State *_out) const = 0;

  /// Identity element for the group
  ///  compose(s, _out) = s
  virtual void getIdentity(State *_out) const = 0;

  /// Inverse element for the group
  /// compose(_in, _out) = getIdentity()
  virtual void getInverse(const State *_in, State *_out) const = 0;

  /// Get the dimension of the state space
  virtual unsigned int getDimension() const = 0;

  /// Copy a state to another.
  virtual void copyState(StateSpace::State *_destination,
                         const StateSpace::State *_source) const = 0;

  /// Exponential mapping of Lie algebra element to a Lie group element.  
  virtual void expMap(
    const Eigen::VectorXd& _tangent, State* _out) const = 0;

  /// Log mapping of Lie group element to a Lie algebra element
  virtual void logMap(const State *_in, Eigen::VectorXd &_tangent) const = 0;
};

using StateSpacePtr = std::shared_ptr<StateSpace>;

}  // namespace statespace
}  // namespace aikido

#endif
