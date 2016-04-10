#include <aikido/statespace/SE2JointStateSpace.hpp>
#include <aikido/statespace/SE2StateSpaceSampleableConstraint.hpp>

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

//=============================================================================
auto SE2JointStateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  Eigen::Vector2d lowerLimits;
  lowerLimits[0] = mJoint->getPositionLowerLimit(0);
  lowerLimits[1] = mJoint->getPositionLowerLimit(1);

  Eigen::Vector2d upperLimits;
  upperLimits[0] = mJoint->getPositionUpperLimit(0);
  upperLimits[1] = mJoint->getPositionUpperLimit(1);

  return std::make_shared<SE2StateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    //std::const_pointer_cast<SE2StateSpace>(shared_from_this()),
    nullptr,
    std::move(_rng), lowerLimits, upperLimits);
}

} // namespace statespace
} // namespace aikido
