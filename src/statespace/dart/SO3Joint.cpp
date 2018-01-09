#include "aikido/statespace/dart/SO3Joint.hpp"

// TODO: This will not be necessary once we switch to using
// BallJoint::convertToRotation instead of calling dart::math::logMap directly.
// See the comment below for more information.
#include <dart/math/Geometry.hpp>

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
  // TODO: We should call BallJoint::convertToRotation instead of logMap once
  // the convertToRotation method is fixed in DART.
  positions = ::dart::math::logMap(
      getQuaternion(static_cast<const State*>(state)).toRotationMatrix());
}

} // namespace dart
} // namespace statespace
} // namespace aikido
