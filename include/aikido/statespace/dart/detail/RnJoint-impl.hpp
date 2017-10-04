#include <aikido/statespace/dart/RnJoint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
extern template class RJoint<0>;

extern template class RJoint<1>;

extern template class RJoint<2>;

extern template class RJoint<3>;

extern template class RJoint<6>;

//==============================================================================
template <int N>
RJoint<N>::RJoint(typename RJoint<N>::DartJoint* _joint)
  : R<N>(), JointStateSpace(_joint)
{
  static_assert(
      N != Eigen::Dynamic,
      "Invalid dimension. Dynamic size dimension is not supported by RnJoint");
}

//==============================================================================
template <int N>
void RJoint<N>::convertPositionsToState(
    const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  this->setValue(static_cast<typename R<N>::State*>(_state), _positions);
}

//==============================================================================
template <int N>
void RJoint<N>::convertStateToPositions(
    const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions = this->getValue(static_cast<const typename R<N>::State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
