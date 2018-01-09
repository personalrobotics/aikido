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
RJoint<N>::RJoint(const typename RJoint<N>::DartJoint* joint)
  : R<N>(), JointStateSpace(joint)
{
  static_assert(
      N != Eigen::Dynamic,
      "Invalid dimension. Dynamic size dimension is not supported by RnJoint");
}

//==============================================================================
template <int N>
void RJoint<N>::convertPositionsToState(
    const Eigen::VectorXd& positions, StateSpace::State* state) const
{
  this->setValue(static_cast<typename R<N>::State*>(state), positions);
}

//==============================================================================
template <int N>
void RJoint<N>::convertStateToPositions(
    const StateSpace::State* state, Eigen::VectorXd& positions) const
{
  positions = this->getValue(static_cast<const typename R<N>::State*>(state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
