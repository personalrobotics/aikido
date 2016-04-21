#include <aikido/statespace/dart/SO2JointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
SO2JointStateSpace::SO2JointStateSpace(
      ::dart::dynamics::SingleDofJoint* _joint)
  : JointStateSpace(_joint)
  , SO2StateSpace()
{
}

//=============================================================================
void SO2JointStateSpace::getState(
  const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  setAngle(static_cast<State*>(_state), _positions[0]);
}

//=============================================================================
void SO2JointStateSpace::setState(
  const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions.resize(1);
  _positions[0] = getAngle(static_cast<const State*>(_state));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
