#include <aikido/statespace/dart/WeldJoint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
WeldJoint::WeldJoint(::dart::dynamics::WeldJoint* _joint)
  : R0(), JointStateSpace(_joint)
{
  // Do nothing.
}

//=============================================================================
void WeldJoint::convertPositionsToState(
  const Eigen::VectorXd& /*_positions*/, StateSpace::State* /*_state*/) const
{
  // Do nothing since the dimension is zero.
}

//=============================================================================
void WeldJoint::convertStateToPositions(
  const StateSpace::State* /*_state*/, Eigen::VectorXd& _positions) const
{
  _positions.resize(Dimension);
}

} // namespace dart
} // namespace statespace
} // namespace aikido

