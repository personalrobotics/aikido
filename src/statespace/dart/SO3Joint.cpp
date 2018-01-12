#include "aikido/statespace/dart/SO3Joint.hpp"

using ::dart::dynamics::BallJoint;

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
SO3Joint::SO3Joint(const ::dart::dynamics::BallJoint* joint)
  : SO3(), JointStateSpace(joint)
{
  // Do nothing.
}

//==============================================================================
void SO3Joint::convertPositionsToState(
    const Eigen::VectorXd& positions, StateSpace::State* state) const
{
  setQuaternion(
      static_cast<State*>(state),
      SO3::Quaternion(BallJoint::convertToRotation(positions)));
}

//==============================================================================
void SO3Joint::convertStateToPositions(
    const StateSpace::State* state, Eigen::VectorXd& positions) const
{
  positions = BallJoint::convertToPositions(
      getQuaternion(static_cast<const State*>(state)).toRotationMatrix());
}

} // namespace dart
} // namespace statespace
} // namespace aikido
