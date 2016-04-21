#include <aikido/statespace/dart/JointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
JointStateSpace::JointStateSpace(::dart::dynamics::Joint* _joint)
  : mJoint(_joint)
{
  assert(_joint);
}

//=============================================================================
::dart::dynamics::Joint* JointStateSpace::getJoint() const
{
  return mJoint;
}

//=============================================================================
void JointStateSpace::getState(StateSpace::State* _state) const
{
  getState(mJoint->getPositions(), _state);
}

//=============================================================================
void JointStateSpace::setState(const StateSpace::State* _state) const
{
  Eigen::VectorXd positions;
  setState(_state, positions);
  mJoint->setPositions(positions);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
