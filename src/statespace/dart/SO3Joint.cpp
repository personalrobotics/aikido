#include <aikido/statespace/dart/SO3Joint.hpp>

// TODO: This will not be necessary once we switch to using
// BallJoint::convertToRotation instead of calling dart::math::logMap directly.
// See the comment below for more information.
#include <dart/math/Geometry.hpp>

using ::dart::dynamics::BallJoint;

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
SO3Joint::SO3Joint(::dart::dynamics::BallJoint* _joint)
  : SO3(), JointStateSpace(_joint)
{
}

//==============================================================================
void SO3Joint::convertPositionsToState(
    const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  setQuaternion(
      static_cast<State*>(_state),
      SO3::Quaternion(BallJoint::convertToRotation(_positions)));
}

//==============================================================================
void SO3Joint::convertStateToPositions(
    const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  // TODO: We should call BallJoint::convertToRotation instead of logMap once
  // the convertToRotation method is fixed in DART.
  _positions = ::dart::math::logMap(
      getQuaternion(static_cast<const State*>(_state)).toRotationMatrix());
}

} // namespace dart
} // namespace statespace
} // namespace aikido
