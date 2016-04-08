#include <aikido/statespace/RealVectorJointStateSpace.hpp>

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

} // namespace statespace
} // namespace aikido
