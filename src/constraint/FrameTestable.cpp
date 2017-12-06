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
    TestableOutcome* outcome) const
{
  DefaultOutcome* defaultOutcomeObject = nullptr;
  if (outcome)
  {
    defaultOutcomeObject = dynamic_cast<DefaultOutcome*>(outcome);
    if (!defaultOutcomeObject)
      throw std::invalid_argument(
          "TestableOutcome pointer is not of type DefaultOutcome.");
  }

  // Set the state
  auto state
      = static_cast<const statespace::dart::MetaSkeletonStateSpace::State*>(
          _state);
  mStateSpace->setState(state);

  // Check the pose constraint
  auto st = mPoseStateSpace->createState();
  mPoseStateSpace->setIsometry(st, mFrame->getTransform());

  bool isSatisfiedResult = mPoseConstraint->isSatisfied(st);
  if (defaultOutcomeObject)
    defaultOutcomeObject->setSatisfiedFlag(isSatisfiedResult);
  return isSatisfiedResult;
}

//==============================================================================
std::unique_ptr<TestableOutcome> FrameTestable::createOutcome() const
{
  return std::unique_ptr<TestableOutcome>(new DefaultOutcome());
}

//==============================================================================
std::shared_ptr<statespace::StateSpace> FrameTestable::getStateSpace() const
{
  return mStateSpace;
}

} // namespace constraint
} // namespace aikido
