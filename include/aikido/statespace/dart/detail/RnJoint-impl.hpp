#include <aikido/statespace/dart/RnJoint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
template <int N>
RnJoint<N>::RnJoint(typename RnJoint<N>::DartJoint* _joint)
  : Rn<N>(), JointStateSpace(_joint)
{
}

//=============================================================================
template <int N>
void RnJoint<N>::convertPositionsToState(
  const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  this->setValue(static_cast<typename Rn<N>::State*>(_state), _positions);
}

//=============================================================================
template <int N>
void RnJoint<N>::convertStateToPositions(
  const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions = this->getValue(
      static_cast<const typename Rn<N>::State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
