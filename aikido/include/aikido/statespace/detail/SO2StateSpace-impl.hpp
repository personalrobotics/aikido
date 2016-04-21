namespace aikido {
namespace statespace {

/// \c StateHandle for a \c SO2StateSpace. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class SO2StateHandle
  : public statespace::StateHandle<SO2StateSpace, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    SO2StateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    SO2StateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    SO2StateSpace, _QualifiedState>::QualifiedState;

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

  /// Gets state as a rotation angle.
  ///
  /// \return rotation angle
  double getAngle() const
  {
    return this->getStateSpace()->getAngle(this->getState());
  }

  /// Sets state to a rotation angle.
  ///
  /// \param _angle rotation angle
  void setAngle(double _angle)
  {
    return this->getStateSpace()->setAngle(this->getState(), _angle);
  }

  /// Gets state as an Eigen transformation.
  ///
  /// \return Eigen transformation
  Eigen::Rotation2Dd getRotation() const
  {
    return this->getStateSpace()->getRotation(this->getState());
  }

  /// Sets state it an Eigen transformation.
  ///
  /// \param _rotation Eigen transformation
  void setRotation(const Eigen::Rotation2Dd& _rotation)
  {
    return this->getStateSpace()->setRotation(this->getState(), _rotation);
  }
};

} // namespace statespace
} // namespace aikido
