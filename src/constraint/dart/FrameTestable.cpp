#include "aikido/constraint/dart/FrameTestable.hpp"

namespace aikido {
namespace constraint {
namespace dart {

//==============================================================================
FrameTestable::FrameTestable(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
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

  mPoseStateSpace = std::dynamic_pointer_cast<const statespace::SE3>(
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
statespace::ConstStateSpacePtr FrameTestable::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
const ::dart::dynamics::BodyNode* cloneFrame(
    const ::dart::dynamics::MetaSkeleton& skeletonClone,
    const ::dart::dynamics::ConstJacobianNodePtr& frame)
{
  using namespace ::dart::dynamics;
  const ConstBodyNodePtr& bodyNodePtr = frame->getBodyNodePtr();
  return skeletonClone.getBodyNode(bodyNodePtr->getName());
}

//==============================================================================
TestablePtr FrameTestable::clone(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  std::cout << "Clone FrameTestable constraint" << std::endl;
  // TODO: assert metaSkeleton is a cloned version of mMetaSkeleton

  auto poseConstraint = std::dynamic_pointer_cast<DartConstraint>(mPoseConstraint);
  if (poseConstraint)
  {
    auto clonedPoseConstraint
        = poseConstraint->clone(collisionDetector, metaSkeleton);
    auto cloned = std::make_shared<FrameTestable>(
      mMetaSkeletonStateSpace,
      metaSkeleton,
      cloneFrame(*metaSkeleton, mFrame),
      clonedPoseConstraint);
    return cloned;
  }

  auto cloned = std::make_shared<FrameTestable>(
      mMetaSkeletonStateSpace,
      metaSkeleton,
      cloneFrame(*metaSkeleton, mFrame),
      mPoseConstraint);

  return cloned;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
