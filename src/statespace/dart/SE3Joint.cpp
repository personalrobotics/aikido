#include "aikido/statespace/dart/SE3Joint.hpp"

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
SE3Joint::SE3Joint(const ::dart::dynamics::FreeJoint* joint)
  : SE3(), JointStateSpace(joint)
{
  // Do nothing.
}

//==============================================================================
void SE3Joint::convertPositionsToState(
    const Eigen::VectorXd& positions, StateSpace::State* state) const
{
  setIsometry(
      static_cast<State*>(state),
      ::dart::dynamics::FreeJoint::convertToTransform(positions));
}

//==============================================================================
void SE3Joint::convertStateToPositions(
    const StateSpace::State* state, Eigen::VectorXd& positions) const
{
  positions = ::dart::dynamics::FreeJoint::convertToPositions(
      getIsometry(static_cast<const SE3::State*>(state)));
}

} // namespace dart
} // namespace statespace
} // namespace aikido
