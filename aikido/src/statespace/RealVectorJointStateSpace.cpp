#include <aikido/statespace/RealVectorJointStateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
RealVectorJointStateSpace::RealVectorJointStateSpace(
      dart::dynamics::Joint* _joint)
  : RealVectorStateSpace(_joint->getNumDofs())
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
