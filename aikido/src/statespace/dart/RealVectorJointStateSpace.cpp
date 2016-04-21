#include <aikido/statespace/dart/RealVectorJointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
RealVectorJointStateSpace::RealVectorJointStateSpace(
      ::dart::dynamics::Joint* _joint)
  : RealVectorStateSpace(_joint->getNumDofs())
  , JointStateSpace(_joint)
{
}

//=============================================================================
void RealVectorJointStateSpace::getState(
  const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  setValue(static_cast<State*>(_state), _positions);
}

//=============================================================================
void RealVectorJointStateSpace::setState(
  const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions = getValue(static_cast<const State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
