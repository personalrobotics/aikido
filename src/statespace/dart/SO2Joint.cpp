#include "aikido/statespace/dart/SO2Joint.hpp"

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
SO2Joint::SO2Joint(
    const ::dart::dynamics::GenericJoint<::dart::math::R1Space>* joint)
  : SO2(), JointStateSpace(joint)
{
  // Do nothing.
}

//==============================================================================
void SO2Joint::convertPositionsToState(
    const Eigen::VectorXd& positions, StateSpace::State* state) const
{
  fromAngle(static_cast<State*>(state), positions[0]);
}

//==============================================================================
void SO2Joint::convertStateToPositions(
    const StateSpace::State* state, Eigen::VectorXd& positions) const
{
  positions.resize(1);
  positions[0] = toAngle(static_cast<const State*>(state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
