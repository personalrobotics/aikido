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

  /// Get the maximum value a call to distance() can return (or an upper bound).
  /// For unbounded state spaces, this function can return infinity.
  virtual double getMaximumExtent() const = 0;

  /// Get a measure of the space (this can be thought of as a generalization of
  /// volume)
  virtual double getMeasure() const = 0;

  /// Copy a state to another.
  virtual void copyState(StateSpace::State *_destination,
                         const StateSpace::State *_source) const = 0;

  /// Computes distance between two states. This function satisfies
  /// the properties of a metric if isMetricSpace() is true, and its
  /// return value will always be between 0 and getMaximumExtent()
  virtual double distance(const StateSpace::State *_state1,
                          const StateSpace::State *_state2) const = 0;

  /// Equal states
  virtual bool equalStates(const StateSpace::State *_state1,
                           const StateSpace::State *_state2) const = 0;

  /// Computes the state that lies at time t in [0, 1] on the segment
  /// that connects from state to to state. The memory location of state
  /// is not required to be different from the memory of either from or to.
  virtual void interpolate(const StateSpace::State *_from,
                           const StateSpace::State *_to, const double _t,
                           StateSpace::State *_state) const = 0;

  /// Exponential mapping of Lie algebra element to a Lie group element.
  virtual void expMap(const Eigen::VectorXd &_tangent, State *_out) const = 0;
};

using StateSpacePtr = std::shared_ptr<StateSpace>;

}  // namespace statespace
}  // namespace aikido

#endif
