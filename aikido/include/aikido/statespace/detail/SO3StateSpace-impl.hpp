namespace aikido {
namespace statespace {

/// \c StateHandle for a \c SO3StateSpace. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class SO3StateHandle
  : public statespace::StateHandle<SO3StateSpace, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    SO3StateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    SO3StateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    SO3StateSpace, _QualifiedState>::QualifiedState;

  using Quaternion = typename State::Quaternion;

  /// Construct and initialize to \c nullptr.
  SO3StateHandle()
  {
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  SO3StateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Constructs a point in SO(3) from a unit quaternion.
  ///
  /// \param _quaternion unit quaternion representing orientation
  const Quaternion& getQuaternion()
  {
    return this->getStateSpace()->getQuaternion(this->getState());
  }

  /// Sets a state to a unit quaternion.
  ///
  /// \param _quaternion unit quaternion representing orientation
  void setQuaternion(const Quaternion& _quaternion)
  {
    this->getStateSpace()->setQuaternion(this->getState(), _quaternion);
  }
};

} // namespace statespace
} // namespace aikido
