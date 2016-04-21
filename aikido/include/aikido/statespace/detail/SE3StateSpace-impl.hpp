namespace aikido {
namespace statespace {

/// \c StateHandle for a \c SE3StateSpace. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class SE3StateHandle
  : public statespace::StateHandle<SE3StateSpace, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    SE3StateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    SE3StateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    SE3StateSpace, _QualifiedState>::QualifiedState;

  /// Construct and initialize to \c nullptr.
  SE3StateHandle()
  {
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  SE3StateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<SE3StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Gets value as an Eigen transformation object.
  ///
  /// \return Eigen transformation
  Eigen::Isometry3d getIsometry() const
  {
    return this->getStateSpace()->getIsometry(this->getState());
  }

  /// Gets value as an Eigen transformation object.
  ///
  /// \return Eigen trasnformation 
  void setIsometry(const Eigen::Isometry3d& _transform) const
  {
    return this->getStateSpace()->setIsometry(this->getState(), _transform);
  }
};

} // namespace statespace
} // namespace aikido
