#include <aikido/statespace/dart/SE3Joint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
SE3Joint::SE3Joint(::dart::dynamics::FreeJoint* _joint)
  : SE3(), JointStateSpace(_joint)
{
  // Do nothing.
}

//==============================================================================
void SE3Joint::convertPositionsToState(
    const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  setIsometry(
      static_cast<State*>(_state),
      ::dart::dynamics::FreeJoint::convertToTransform(_positions));
}

//==============================================================================
void SE3Joint::convertStateToPositions(
    const StateSpace::State* _state, Eigen::VectorXd& _positions) const
{
  _positions = ::dart::dynamics::FreeJoint::convertToPositions(
      getIsometry(static_cast<const SE3::State*>(_state)));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
