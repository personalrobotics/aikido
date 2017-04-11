#include <aikido/statespace/dart/RnJoint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
extern template
class RnJoint<0>;

extern template
class RnJoint<1>;

extern template
class RnJoint<2>;

extern template
class RnJoint<3>;

extern template
class RnJoint<6>;

//=============================================================================
template <int N>
RnJoint<N>::RnJoint(typename RnJoint<N>::DartJoint* _joint)
  : R<N>(), JointStateSpace(_joint)
{
  // Do nothing
}

//=============================================================================
template <int N>
void RnJoint<N>::convertPositionsToState(
  const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  this->setValue(static_cast<typename R<N>::State*>(_state), _positions);
}

//=============================================================================
template <int N>
void RnJoint<N>::convertStateToPositions(
  const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions = this->getValue(
      static_cast<const typename R<N>::State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
