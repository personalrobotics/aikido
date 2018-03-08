#include "aikido/constraint/dart/FrameTestable.hpp"

namespace aikido {
namespace constraint {
namespace dart {

//==============================================================================
FrameTestable::FrameTestable(
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    ::dart::dynamics::MetaSkeletonPtr _metaskeleton,
    ::dart::dynamics::ConstJacobianNodePtr _frame,
    TestablePtr _poseConstraint)
  : mMetaSkeletonStateSpace(std::move(_metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(_metaskeleton))
  , mFrame(_frame)
  , mPoseConstraint(std::move(_poseConstraint))
  , mPoseStateSpace()
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("_metaskeleton is nullptr.");

  if (!mPoseConstraint)
    throw std::invalid_argument("_poseConstraint is nullptr.");

  if (!mFrame)
    throw std::invalid_argument("_frame is nullptr.");

  mPoseStateSpace = std::dynamic_pointer_cast<statespace::SE3>(
      mPoseConstraint->getStateSpace());

  if (!mPoseStateSpace)
    throw std::invalid_argument("_poseConstraint is not in SE3.");

  // TODO: If possible, check that _frame is influenced by at least
  // one DegreeOfFreedom in the _metaSkeletonStateSpace's Skeleton.
}

//==============================================================================
bool FrameTestable::isSatisfied(
    const statespace::StateSpace::State* _state, TestableOutcome* outcome) const
{
  // Set the state
  auto state
      = static_cast<const statespace::dart::MetaSkeletonStateSpace::State*>(
          _state);
  mMetaSkeletonStateSpace->setState(mMetaSkeleton.get(), state);

  // Check the pose constraint
  auto st = mPoseStateSpace->createState();
  mPoseStateSpace->setIsometry(st, mFrame->getTransform());
  return mPoseConstraint->isSatisfied(st, outcome);
}

//==============================================================================
std::unique_ptr<TestableOutcome> FrameTestable::createOutcome() const
{
  return mPoseConstraint->createOutcome();
}

//==============================================================================
std::shared_ptr<statespace::StateSpace> FrameTestable::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
