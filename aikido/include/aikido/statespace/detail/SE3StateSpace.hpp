namespace aikido {
namespace statespace {

template <class _QualifiedState>
class SE3StateHandle
  // TODO: This should inherit from CompoundStateSpace::Handle.
  : public statespace::StateHandle<SE3StateSpace, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    SE3StateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    SE3StateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    SE3StateSpace, _QualifiedState>::QualifiedState;

  SE3StateHandle()
  {
  }

  SE3StateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<SE3StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Gets value as a transformation.
  Eigen::Isometry3d getIsometry() const
  {
    return this->getStateSpace()->getIsometry(this->getState());
  }

  /// Sets value to a transformation.
  void setIsometry(const Eigen::Isometry3d& _transform) const
  {
    return this->getStateSpace()->setIsometry(this->getState(), _transform);
  }
};

} // namespace statespace
} // namespace aikido
