#include <aikido/statespace/dart/SO2Joint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
SO2Joint::SO2Joint(
    ::dart::dynamics::GenericJoint<::dart::math::R1Space>* _joint)
  : SO2(), JointStateSpace(_joint)
{
  // Do nothing.
}

//==============================================================================
void SO2Joint::convertPositionsToState(
    const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  setAngle(static_cast<State*>(_state), _positions[0]);
}

//==============================================================================
void SO2Joint::convertStateToPositions(
    const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions.resize(1);
  _positions[0] = getAngle(static_cast<const State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
