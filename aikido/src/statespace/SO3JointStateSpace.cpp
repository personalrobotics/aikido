#include <aikido/statespace/SO3JointStateSpace.hpp>
#include <aikido/statespace/SO3StateSpaceSampleableConstraint.hpp>

// TODO: This will not be necessary once we switch to using
// BallJoint::convertToRotation instead of calling dart::math::logMap directly.
// See the comment below for more information.
#include <dart/math/Geometry.h>

using dart::dynamics::BallJoint;

namespace aikido {
namespace statespace {

//=============================================================================
SO3JointStateSpace::SO3JointStateSpace(dart::dynamics::BallJoint* _joint)
  : JointStateSpace(_joint)
  , SO3StateSpace()
{
}

//=============================================================================
void SO3JointStateSpace::getState(StateSpace::State* _state) const
{
  setQuaternion(static_cast<State*>(_state), SO3StateSpace::Quaternion(
    BallJoint::convertToRotation(mJoint->getPositions())));
}

//=============================================================================
void SO3JointStateSpace::setState(const StateSpace::State* _state) const
{
  // TODO: We should call BallJoint::convertToRotation instead of logMap once
  // the convertToRotation method is fixed in DART.
  mJoint->setPositions(
    dart::math::logMap(
      getQuaternion(static_cast<const State*>(_state)).toRotationMatrix()));
}

//=============================================================================
auto SO3JointStateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  for (size_t i = 0; i < 3; ++i)
  {
    if (mJoint->hasPositionLimit(i))
      throw std::runtime_error(
        "Position limits are unsupported on joints with SO(3) topology.");
  }

  return std::make_shared<SO3StateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    std::const_pointer_cast<SO3JointStateSpace>(shared_from_this()),
    std::move(_rng));
}


} // namespace statespace
} // namespace aikido
