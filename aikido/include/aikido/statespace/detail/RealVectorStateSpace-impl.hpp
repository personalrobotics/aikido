#include <type_traits>

namespace aikido {
namespace statespace {

/// \c StateHandle for a \c RealVectorStateSpace. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
///
/// \tparam _QualifiedState type of \c State being wrapped
template <class _QualifiedState>
class RealVectorStateHandle 
  : public statespace::StateHandle<RealVectorStateSpace, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    RealVectorStateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    RealVectorStateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    RealVectorStateSpace, _QualifiedState>::QualifiedState;

  using ValueType = std::conditional<std::is_const<QualifiedState>::value,
    const Eigen::VectorXd, Eigen::VectorXd>;

  /// Construct and initialize to \c nullptr.
  RealVectorStateHandle()
  {
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  RealVectorStateHandle(const StateSpace* _space, QualifiedState* _state)
    : statespace::StateHandle<StateSpace, QualifiedState>(_space, _state)
  {
  }

  /// Gets the real vector stored in this state.
  ///
  /// \return real vector stored in this state
  Eigen::Map<const Eigen::VectorXd> getValue() 
  {
    return this->getStateSpace()->getValue(this->getState());
  }

  /// Sets the real vector stored in this state.
  ///
  /// \param _value real vector to store in \c _state
  void setValue(const Eigen::VectorXd& _value)
  {
    return this->getStateSpace()->setValue(this->getState(), _value);
  }
};

} // namespace statespace
} // namespace aikido
