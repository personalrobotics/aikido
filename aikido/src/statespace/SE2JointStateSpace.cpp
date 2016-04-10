#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SE2JointStateSpace.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SE2JointStateSpace::SE2JointStateSpace(dart::dynamics::PlanarJoint* _joint)
  : JointStateSpace(_joint)
  , SE2StateSpace()
{
}

//=============================================================================
void SE2JointStateSpace::getState(StateSpace::State* _state) const
{
  Isometry2d pose = Isometry2d::Identity();
  pose.rotate(Eigen::Rotation2Dd(mJoint->getPosition(2)));
  pose.pretranslate(Eigen::Vector2d(
    mJoint->getPosition(0), mJoint->getPosition(1)));

  setIsometry(static_cast<SE2StateSpace::State*>(_state), pose);
}

//=============================================================================
void SE2JointStateSpace::setState(const StateSpace::State* _state) const
{
  auto pose = getIsometry(static_cast<const SE2StateSpace::State*>(_state));

  Eigen::Rotation2Dd rotation(0);
  rotation.fromRotationMatrix(pose.rotation());

  mJoint->setPosition(0, pose.translation()[0]);
  mJoint->setPosition(1, pose.translation()[1]);
  mJoint->setPosition(2, rotation.angle());
}

} // namespace statespace
} // namespace aikido
