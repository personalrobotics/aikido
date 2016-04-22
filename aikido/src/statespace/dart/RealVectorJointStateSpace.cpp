#include <aikido/statespace/dart/RealVectorJointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
RealVectorJointStateSpace::RealVectorJointStateSpace(
      ::dart::dynamics::Joint* _joint)
  : Rn(_joint->getNumDofs())
  , JointStateSpace(_joint)
{
}

//=============================================================================
void RealVectorJointStateSpace::convertPositionsToState(
  const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  setValue(static_cast<State*>(_state), _positions);
}

//=============================================================================
void RealVectorJointStateSpace::convertStateToPositions(
  const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions = getValue(static_cast<const State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
