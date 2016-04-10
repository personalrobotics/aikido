#include <aikido/statespace/SE3JointStateSpace.hpp>

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
  throw std::runtime_error(
    "SE3StateSpace::createSampleableConstraint is not implemented.");
}

} // namespace statespace
} // namespace aikido
