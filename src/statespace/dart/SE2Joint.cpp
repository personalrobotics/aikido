#include <aikido/statespace/dart/SE2Joint.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//=============================================================================
SE2Joint::SE2Joint(::dart::dynamics::PlanarJoint* _joint)
  : SE2()
  , JointStateSpace(_joint)
{
}

//=============================================================================
void SE2Joint::convertPositionsToState(
  const Eigen::VectorXd& _positions, StateSpace::State* _state) const
{
  Isometry2d pose = Isometry2d::Identity();
  pose.rotate(Eigen::Rotation2Dd(_positions[2]));
  pose.pretranslate(_positions.head<2>());

  setIsometry(static_cast<SE2::State*>(_state), pose);
}

//=============================================================================
void SE2Joint::convertStateToPositions(
  const StateSpace::State* _state,
  Eigen::VectorXd& _positions) const
{
  auto pose = getIsometry(static_cast<const SE2::State*>(_state));

  Eigen::Rotation2Dd rotation(0);
  rotation.fromRotationMatrix(pose.rotation());

  _positions.resize(3);
  _positions.head<2>() = pose.translation();
  _positions[2] = rotation.angle();
}

} // namespace dart
} // namespace statespace
} // namespace aikido
