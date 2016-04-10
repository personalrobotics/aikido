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
  Eigen::Vector2d lowerLimits, upperLimits;

  for (size_t i = 0; i < 2; ++i)
  {
    lowerLimits[i] = mJoint->getPositionLowerLimit(i);
    upperLimits[i] = mJoint->getPositionUpperLimit(i);
  }

  if (mJoint->hasPositionLimit(2))
  {
    throw std::runtime_error(
      "Position limits are unsupported on the rotation component of joints"
      " with SE(2) topology.");
  }

  return std::make_shared<SE2StateSpaceSampleableConstraint>(
    // TODO: SampleableConstraint should operate on `const StateSpace`.
    std::const_pointer_cast<SE2JointStateSpace>(shared_from_this()),
    std::move(_rng), lowerLimits, upperLimits);
}

} // namespace statespace
} // namespace aikido
