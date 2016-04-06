namespace aikido {
namespace statespace {

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

  SO2StateHandle()
  {
  }

  SO2StateHandle(const StateSpace* _space, State* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Gets the angle of the rotation encoded by this state.
  double getAngle() const
  {
    return this->getStateSpace()->getAngle(this->getState());
  }

  /// Sets this state to a rotation by the specified angle.
  void setAngle(double _angle)
  {
    return this->getStateSpace()->setAngle(this->getState(), _angle);
  }

  /// Gets value as a rigid body rotation.
  Eigen::Rotation2Dd getRotation() const
  {
    return this->getStateSpace()->getRotation(this->getState());
  }

  /// Sets this state to the given rotation.
  void setRotation(const Eigen::Rotation2Dd& _rotation)
  {
    return this->getStateSpace()->setRotation(this->getState(), _rotation);
  }
};

} // namespace statespace
} // namespace aikido
