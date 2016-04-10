#include <aikido/statespace/SE3JointStateSpace.hpp>
#include <aikido/statespace/SE3StateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SE3JointStateSpace::SE3JointStateSpace(dart::dynamics::FreeJoint* _joint)
  : JointStateSpace(_joint)
  , SE3StateSpace()
{
}

//=============================================================================
void SE3JointStateSpace::getState(StateSpace::State* _state) const
{
  setIsometry(static_cast<State*>(_state),
    dart::dynamics::FreeJoint::convertToTransform(
      mJoint->getPositions()));
}

//=============================================================================
void SE3JointStateSpace::setState(const StateSpace::State* _state) const
{
  mJoint->setPositions(
    dart::dynamics::FreeJoint::convertToPositions(
      getIsometry(static_cast<const SE3StateSpace::State*>(_state))));
}

//=============================================================================
auto SE3JointStateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  for (size_t i = 0; i < 3; ++i)
  {
    if (mJoint->hasPositionLimit(i))
      throw std::runtime_error(
        "Position limits are unsupported on the rotation components of joints"
        " with SE(3) topology.");
  }

  Eigen::Vector3d lowerLimits, upperLimits;
  for (size_t i = 0; i < 3; ++i)
  {
    lowerLimits[i] = mJoint->getPositionLowerLimit(i + 3);
    upperLimits[i] = mJoint->getPositionUpperLimit(i + 3);
  }

  return std::make_shared<SE3StateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    std::const_pointer_cast<SE3JointStateSpace>(shared_from_this()),
    std::move(_rng), lowerLimits, upperLimits);
}

} // namespace statespace
} // namespace aikido
