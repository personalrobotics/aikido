#ifndef AIKIDO_STATESPACE_STATEHANDLE_HPP_
#define AIKIDO_STATESPACE_STATEHANDLE_HPP_

#include <type_traits>

namespace aikido {
namespace statespace {

/// Wrap a State with its StateSpace to provide convenient accessor methods.
/// The template parameter \c _QualifiedState is necessary to support both
/// \c const and non-<tt>const</tt> states.
///
/// \tparam _StateSpace type of \c StateSpace this state is a member of
/// \tparam _QualifiedState type of \c State being wrapped
template <class _StateSpace, class _QualifiedState>
class StateHandle
{
public:
  using StateSpace = _StateSpace;
  using State = typename StateSpace::State;
  using QualifiedState = _QualifiedState;

  /// Constructs a nullptr handle.
  StateHandle();

  /// Wrap state, which must be form the provided StateSpace.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  StateHandle(const StateSpace* _space, QualifiedState* _state);

  StateHandle(const StateHandle&) = default;
  StateHandle(StateHandle&&) = default;

  StateHandle& operator=(StateHandle&&) = default;
  StateHandle& operator=(const StateHandle&) = default;

  /// Implicitly convert to a \c State pointer.
  operator QualifiedState*() const;

  /// Reset StateHandle to nullptr.
  void reset();

  /// Reset the state, which must be from the provided StateSpace.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  void reset(const StateSpace* _space, QualifiedState* _state);

  template <typename Q = QualifiedState>
  typename std::enable_if<std::is_const<Q>::value, Q*>::type // only for const X
  getState() const { return mState; }

  /// Gets the State.
  ///
  /// \return state wrapped by this handle
  template <typename Q = QualifiedState>
  typename std::enable_if<!std::is_const<Q>::value, Q*>::type // only for non-const X
  getState() { return mState; }

  /// Gets the State.
  ///
  /// \return state wrapped by this handle
  template <typename Q = QualifiedState>
  typename std::enable_if<!std::is_const<Q>::value, const Q*>::type // only for non-const X
  getState() const { return mState; }

  /// Gets the state space that created this state.
  ///
  /// \return state space created this state
  const StateSpace* getStateSpace() const;

protected:
  const StateSpace* mSpace;
  QualifiedState* mState;
};

} // namespace statespace
} // namespace aikido

#include "detail/StateHandle-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_STATEHANDLE_HPP_
