#include <aikido/statespace/SO2JointStateSpace.hpp>
#include <aikido/statespace/SO2StateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SO2JointStateSpace::SO2JointStateSpace(dart::dynamics::SingleDofJoint* _joint)
  : JointStateSpace(_joint)
  , SO2StateSpace()
{
}

//=============================================================================
void SO2JointStateSpace::getState(StateSpace::State* _state) const
{
  setAngle(static_cast<State*>(_state), mJoint->getPosition(0));
}

//=============================================================================
void SO2JointStateSpace::setState(const StateSpace::State* _state) const
{
  mJoint->setPosition(0, getAngle(static_cast<const State*>(_state)));
}

//=============================================================================
auto SO2JointStateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  return std::make_shared<SO2StateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    // std::const_pointer_cast<SO2StateSpace>(shared_from_this()),
    nullptr,
    std::move(_rng));
}


} // namespace statespace
} // namespace aikido
