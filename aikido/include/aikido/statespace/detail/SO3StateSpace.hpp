namespace aikido {
namespace statespace {

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

  SO3StateHandle()
  {
  }

  SO3StateHandle(const StateSpace* _space, State* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Gets value as a transform.
  const Quaternion& getQuaternion() const
  {
    return this->getStateSpace()->getQuaternion(this->getState());
  }

  /// Sets value to a transform.
  void setQuaternion(const Quaternion& _quaternion)
  {
    this->getStateSpace()->setQuaternion(this->getState(), _quaternion);
  }
};

} // namespace statespace
} // namespace aikido
