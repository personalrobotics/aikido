#include "aikido/statespace/dart/WeldJoint.hpp"

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
WeldJoint::WeldJoint(const ::dart::dynamics::WeldJoint* joint)
  : R0(), JointStateSpace(joint)
{
  // Do nothing.
}

//==============================================================================
void WeldJoint::convertPositionsToState(
    const Eigen::VectorXd& /*positions*/, StateSpace::State* /*state*/) const
{
  // Do nothing since the dimension is zero.
}

//==============================================================================
void WeldJoint::convertStateToPositions(
    const StateSpace::State* /*state*/, Eigen::VectorXd& positions) const
{
  positions.resize(DimensionAtCompileTime);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
