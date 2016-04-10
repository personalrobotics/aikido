#include <aikido/statespace/RealVectorJointStateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {
namespace {

RealVectorStateSpace::Bounds getJointBounds(dart::dynamics::Joint* _joint)
{
  RealVectorStateSpace::Bounds bounds(_joint->getNumDofs(), 2);

  for (size_t idof = 0; idof < bounds.rows(); ++idof)
  {
    bounds(idof, 0) = _joint->getPositionLowerLimit(idof);
    bounds(idof, 1) = _joint->getPositionUpperLimit(idof);
  }

  return bounds;
}

} // namespace

//=============================================================================
RealVectorJointStateSpace::RealVectorJointStateSpace(
      dart::dynamics::Joint* _joint)
  : RealVectorStateSpace(getJointBounds(_joint))
  , JointStateSpace(_joint)
{
}

//=============================================================================
void RealVectorJointStateSpace::getState(StateSpace::State* _state) const
{
  setValue(static_cast<State*>(_state), mJoint->getPositions());
}

//=============================================================================
void RealVectorJointStateSpace::setState(const StateSpace::State* _state) const
{
  mJoint->setPositions(getValue(static_cast<const State*>(_state)));
}

//=============================================================================
auto RealVectorJointStateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  return std::make_shared<RealVectorStateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    //std::const_pointer_cast<RealVectorStateSpace>(shared_from_this()),
    nullptr, // TODO: enable_shared_from_this
    std::move(_rng));
}

} // namespace statespace
} // namespace aikido
