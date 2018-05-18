namespace aikido {
namespace statespace {

/// \c StateHandle for a \c SO2. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class SO2StateHandle : public statespace::StateHandle<SO2, _QualifiedState>
{
public:
  using typename statespace::StateHandle<SO2, _QualifiedState>::State;
  using typename statespace::StateHandle<SO2, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<SO2, _QualifiedState>::QualifiedState;

  /// Construct and initialize to \c nullptr.
  SO2StateHandle()
  {
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  SO2StateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Returns angle corresponding to the state.
  double toAngle() const
  {
    return this->getStateSpace()->toAngle(this->getState());
  }

  /// Sets state given a rotation angle.
  ///
  /// \param[in] angle rotation angle.
  void fromAngle(double angle)
  {
    return this->getStateSpace()->fromAngle(this->getState(), angle);
  }

  /// Returns the Eigen transformation corresponding to state.
  Eigen::Rotation2Dd toRotation() const
  {
    return this->getStateSpace()->toRotation(this->getState());
  }

  /// Sets state given an Eigen transformation.
  ///
  /// \param[in] rotation Eigen transformation.
  void fromRotation(const Eigen::Rotation2Dd& rotation)
  {
    return this->getStateSpace()->fromRotation(this->getState(), rotation);
  }
};

} // namespace statespace
} // namespace aikido
