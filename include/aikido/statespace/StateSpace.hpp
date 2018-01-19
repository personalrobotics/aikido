#ifndef AIKIDO_STATESPACE_STATESPACE_HPP_
#define AIKIDO_STATESPACE_STATESPACE_HPP_

#include <memory>
#include <Eigen/Dense>
#include "aikido/common/RNG.hpp"
#include "aikido/statespace/ScopedState.hpp"

namespace aikido {
namespace statespace {

/// Represents a Lie group and its associated Lie algebra, i.e. a
/// differentiable manifold embedded in Euclidean space. This is a base class
/// for all other state spaces and provides the following operations:
///
/// - a group operation
/// - an identity element
/// - an inverse operation
/// - the log map from an element of the Lie group to the Lie algebra
/// - the exponential map from an element of the Lie algebra to the Lie group
///
/// These operations on \c StateSpace are \b only defined on on \c State
/// objects created by the \b same \c StateSpace instance. \c State is an
/// opaque class that can only be modified if you know which concrete type
/// of \c StateSpace created it. We \b strongly recommend using the
/// \c ScopedState and \c StateHandle mechanism to keep each \c State paired
/// with the \c StateSpace that it resides in.
class StateSpace
{
public:
  /// Base class for all States.
  class State
  {
  protected:
    /// This is a base class that should only only be used in derived classes.
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

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Allocate a new state. This must be deleted with \c freeState. This is a
  /// helper function that allocates memory, uses \c allocateStateInBuffer to
  /// create a \c State, and returns that pointer.
  ///
  /// \return state in this space
  virtual State* allocateState() const;

  /// Free a state previously created by \c allocateState. It is undefined
  /// behavior to access \c _state after calling this function.
  ///
  /// \param _state state to be deleted
  virtual void freeState(State* _state) const;

  /// Gets the size of a State, in bytes.
  ///
  /// \return size, in bytes, requires to store a \c State
  virtual std::size_t getStateSizeInBytes() const = 0;

  /// Create a new state in a pre-allocated buffer. The input argument must
  /// contain at least \c getStateSizeInBytes() bytes of memory. This state
  /// must be freed with \c freeStateInBuffer before freeing \c _buffer.
  ///
  /// \param _buffer memory used to store the returned state
  /// \return state object allocated in \c _buffer
  virtual State* allocateStateInBuffer(void* _buffer) const = 0;

  /// Free a state previously created by \c allocateStateInBuffer. It is
  /// undefined behavior to access \c _state after calling this function.
  ///
  /// \param _state state to free
  virtual void freeStateInBuffer(State* _state) const = 0;

  /// Lie group operation for this StateSpace. It is not acceptable for \c _out
  /// to share memory with \c _state1 or \c _state2.
  ///
  /// \param _state1 left input state
  /// \param _state2 right input state
  /// \param[out] _out output state
  virtual void compose(
      const State* _state1, const State* _state2, State* _out) const = 0;

  /// Lie group operation for this StateSpace. This is an in-place version of
  /// the three argument \c compose member function that computes:
  ///
  /// \code
  /// _state1 = _state1 * _state2.
  /// \endcode
  ///
  /// \param[in,out] _state1 left input state, overwritten by output
  /// \param _state2 right input state
  virtual void compose(State* _state1, const State* _state2) const;

  /// Gets the identity element for this Lie group, such that:
  /// \code
  /// compose(state, identity) = state
  /// \endcode
  ///
  /// \param[out] _out output state
  virtual void getIdentity(State* _out) const = 0;

  /// Gets the inverse of \c _in in this Lie group, such that:
  /// \code
  /// compose(state, inverse(state)) = identity
  /// \endcode
  /// It is not acceptable for \c _in to share memory with \c _out.
  ///
  /// \param _state input state
  /// \param[out] _out output state
  virtual void getInverse(const State* _state, State* _out) const = 0;

  /// Gets the inverse of \c _in in this Lie group. This is an in-place
  /// version of the two-argument \c getInverse member function.
  ///
  /// \param[in,out] _state input state, to be overwritten by output
  virtual void getInverse(State* _state) const;

  /// Get the dimension of this Lie group. This is also the dimension of the
  /// tangent space, i.e. the Lie algebra, associated with this group.
  ///
  /// \return dimension of this state space
  virtual std::size_t getDimension() const = 0;

  /// Copy a state.
  ///
  /// \param _source input state
  /// \param[out] _destination output state
  virtual void copyState(
      const StateSpace::State* _source,
      StateSpace::State* _destination) const = 0;

  /// Exponential mapping of Lie algebra element to a Lie group element. The
  /// parameterization of the tangent space is defined by the concrete
  /// implementation of this class.
  ///
  /// \param _state element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  virtual void expMap(const Eigen::VectorXd& _tangent, State* _out) const = 0;

  /// Log mapping of Lie group element to a Lie algebra element. The
  /// parameterization of the tangent space is defined by the concrete
  /// implementation of this class.
  ///
  /// \param _state element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  virtual void logMap(const State* _in, Eigen::VectorXd& _tangent) const = 0;

  /// Print the state to the output stream
  /// \param _state The element to print
  /// \param _os The stream to print to
  virtual void print(const State* _state, std::ostream& _os) const = 0;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_STATESPACE_HPP_
