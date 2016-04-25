namespace aikido {
namespace statespace {

/// \c StateHandle for a \c SE2. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class SE2StateHandle
  : public statespace::StateHandle<SE2, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    SE2, _QualifiedState>::State;
  using typename statespace::StateHandle<
    SE2, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    SE2, _QualifiedState>::QualifiedState;

  /// Construct and initialize to \c nullptr.
  SE2StateHandle()
  {
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  SE2StateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<SE2, QualifiedState>(_space, _state)
  {
  }

  /// Gets value as an Eigen transformation object.
  ///
  /// \return Eigen transformation
  Eigen::Isometry2d getIsometry() const
  {
    return this->getStateSpace()->getIsometry(this->getState());
  }

  /// Sets value to an Eigen transfomation object.
  ///
  /// \param _transform Eigen transformation
  void setIsometry(const Eigen::Isometry2d& _transform) const
  {
    return this->getStateSpace()->setIsometry(this->getState(), _transform);
  }
};

} // namespace statespace
} // namespace aikido
