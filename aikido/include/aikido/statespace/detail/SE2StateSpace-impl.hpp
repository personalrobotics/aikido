namespace aikido {
namespace statespace {

template <class _QualifiedState>
class SE2StateHandle
  // TODO: This should inherit from CompoundStateSpace::Handle.
  : public statespace::StateHandle<SE2StateSpace, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    SE2StateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    SE2StateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    SE2StateSpace, _QualifiedState>::QualifiedState;

  SE2StateHandle()
  {
  }

  SE2StateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<SE2StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Gets value as a transformation.
  Eigen::Isometry2d getIsometry() const
  {
    return this->getStateSpace()->getIsometry(this->getState());
  }

  /// Sets value to a transformation.
  void setIsometry(const Eigen::Isometry2d& _transform) const
  {
    return this->getStateSpace()->setIsometry(this->getState(), _transform);
  }
};

} // namespace statespace
} // namespace aikido
