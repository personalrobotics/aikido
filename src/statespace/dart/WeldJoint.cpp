#include <aikido/statespace/dart/WeldJoint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
WeldJoint::WeldJoint(
      ::dart::dynamics::WeldJoint* _joint)
  : Rn(_joint->getNumDofs())
  , JointStateSpace(_joint)
{
}

//=============================================================================
void WeldJoint::convertPositionsToState(
  const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  setValue(static_cast<State*>(_state), _positions);
}

//=============================================================================
void WeldJoint::convertStateToPositions(
  const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions = getValue(static_cast<const State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido

