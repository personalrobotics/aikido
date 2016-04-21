#include <aikido/statespace/dart/SE3JointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
SE3JointStateSpace::SE3JointStateSpace(::dart::dynamics::FreeJoint* _joint)
  : JointStateSpace(_joint)
  , SE3StateSpace()
{
}

//=============================================================================
void SE3JointStateSpace::convertPositionsToState(
  const Eigen::VectorXd& _positions,
  StateSpace::State* _state) const
{
  setIsometry(static_cast<State*>(_state),
    ::dart::dynamics::FreeJoint::convertToTransform(_positions));
}

//=============================================================================
void SE3JointStateSpace::convertStateToPositions(
  const StateSpace::State* _state,
  Eigen::VectorXd& _positions) const
{
  _positions = ::dart::dynamics::FreeJoint::convertToPositions(
      getIsometry(static_cast<const SE3StateSpace::State*>(_state)));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
