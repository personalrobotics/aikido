#ifndef AIKIDO_STATESPACE_STATEHANDLE_HPP_
#define AIKIDO_STATESPACE_STATEHANDLE_HPP_

#include <type_traits>

namespace aikido {
namespace statespace {

/// Wrap a State with its StateSpace to provide convenient accessor methods.
/// The template parameter \c _QualifiedState is necessary to support both
/// \c const and non-<tt>const</tt> states.
///
/// \tparam _StateSpace Type of \c StateSpace this state is a member of
/// \tparam _QualifiedState Type of \c State being wrapped
template <class _StateSpace, class _QualifiedState>
class StateHandle
{
public:
  using StateSpace = _StateSpace;

  using QualifiedState = _QualifiedState;

  using State = typename StateSpace::State;
  using ConstState =
      typename std::conditional<std::is_const<QualifiedState>::value,
                                QualifiedState,
                                const QualifiedState>::type;

  using NonConstHandle = StateHandle<StateSpace, State>;
  using ConstHandle = StateHandle<StateSpace, ConstState>;

  /// Constructs a nullptr handle.
  StateHandle();

  /// Wrap state, which must be form the provided StateSpace.
  ///
  /// \param space State space that created \c state.
  /// \param state State created by \c space.
  StateHandle(const StateSpace* space, QualifiedState* state);

  StateHandle(const StateHandle&) = default;
  StateHandle(StateHandle&&) = default;

  StateHandle& operator=(const StateHandle&) = default;
  StateHandle& operator=(StateHandle&&) = default;

  /// Implicitly convert to a \c State pointer.
  operator QualifiedState*() const;

  /// Resets StateHandle to nullptr.
  void reset();

  /// Resets the state, which must be from the provided StateSpace.
  ///
  /// \param space State space that created \c state.
  /// \param state State created by \c space.
  void reset(const StateSpace* space, QualifiedState* state);

  /// Returns the State. This function is enabled only if QualifiedState is a
  /// non-const State type.
  ///
  /// \return state wrapped by this handle
  template <typename Q = QualifiedState>
  auto getState() ->
      typename std::enable_if<!std::is_const<Q>::value, Q*>::type;
  // Note: We don't define non-const function for const State type because it
  // violates const-correctness.

  /// Returns the State.
  ///
  /// \return State wrapped by this handle
  template <typename Q = QualifiedState>
  auto getState() const ->
      typename std::conditional<std::is_const<Q>::value, Q*, const Q*>::type;

  /// Returns the state space that created this state.
  ///
  /// \return State space created this state
  const StateSpace* getStateSpace() const;

protected:
  /// State space of the sate that is managed by this handler.
  const StateSpace* mSpace;

  /// State managed by this handler. This can be either const or non-const type.
  QualifiedState* mState;
};

} // namespace statespace
} // namespace aikido

#include "detail/StateHandle-impl.hpp"

#endif // AIKIDO_STATESPACE_STATEHANDLE_HPP_
