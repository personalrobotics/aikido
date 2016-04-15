#include <aikido/constraint/FkTestable.hpp>

namespace aikido
{
namespace constraint
{
FkTestable::FkTestable(statespace::MetaSkeletonStateSpacePtr _stateSpace,
                       dart::dynamics::Frame* _frame,
                       TestableConstraintPtr _poseConstraint)
    : mStateSpace(std::move(_stateSpace))
    , mFrame(_frame)
    , mPoseConstraint(std::move(_poseConstraint))
    , mPoseStateSpace(std::dynamic_pointer_cast<statespace::SE3StateSpace>(
          mPoseConstraint->getStateSpace()))
{
  assert(mPoseStateSpace);
}

bool FkTestable::isSatisfied(const statespace::StateSpace::State* _state) const
{
  // Set the state
  auto state = static_cast<const statespace::MetaSkeletonStateSpace::State*>(_state);
  mStateSpace->setStateOnMetaSkeleton(state);

  // Perform FK
  Eigen::Isometry3d pose = mFrame->getTransform();

  // Check the pose constraint
  auto st = mPoseStateSpace->createState();
  mPoseStateSpace->setIsometry(st, mFrame->getTransform());

  return mPoseConstraint->isSatisfied(st);
}

std::shared_ptr<statespace::StateSpace> FkTestable::getStateSpace() const
{
  return mStateSpace;
}
}
}
