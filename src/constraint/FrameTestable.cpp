#include <aikido/constraint/FrameTestable.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
FrameTestable::FrameTestable(
    statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::ConstJacobianNodePtr _frame,
    TestablePtr _poseConstraint)
  : mStateSpace(std::move(_stateSpace))
  , mFrame(_frame)
  , mPoseConstraint(std::move(_poseConstraint))
  , mPoseStateSpace()
{
  if (!mStateSpace)
    throw std::invalid_argument("_stateSpace is nullptr.");

  if (!mPoseConstraint)
    throw std::invalid_argument("_poseConstraint is nullptr.");

  if (!mFrame)
    throw std::invalid_argument("_frame is nullptr.");

  mPoseStateSpace = std::dynamic_pointer_cast<statespace::SE3>(
      mPoseConstraint->getStateSpace());

  if (!mPoseStateSpace)
    throw std::invalid_argument("_poseConstraint is not in SE3.");

  // TODO: If possible, check that _frame is influenced by at least
  // one DegreeOfFreedom in the _stateSpace's Skeleton.
}

//==============================================================================
bool FrameTestable::isSatisfied(
    const statespace::StateSpace::State* _state,
    TestableOutcome* _outcome) const
{
  // Set the state
  auto state
      = static_cast<const statespace::dart::MetaSkeletonStateSpace::State*>(
          _state);
  mStateSpace->setState(state);

  // Check the pose constraint
  auto st = mPoseStateSpace->createState();
  mPoseStateSpace->setIsometry(st, mFrame->getTransform());

  return mPoseConstraint->isSatisfied(st);
}

//==============================================================================
std::shared_ptr<statespace::StateSpace> FrameTestable::getStateSpace() const
{
  return mStateSpace;
}

} // namespace constraint
} // namespace aikido
