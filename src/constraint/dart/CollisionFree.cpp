#include "aikido/constraint/dart/CollisionFree.hpp"

namespace aikido {
namespace constraint {
namespace dart {

//==============================================================================
CollisionFree::CollisionFree(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    ::dart::dynamics::MetaSkeletonPtr _metaskeleton,
    std::shared_ptr<::dart::collision::CollisionDetector> _collisionDetector,
    ::dart::collision::CollisionOption _collisionOptions)
  : mMetaSkeletonStateSpace(std::move(_metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(_metaskeleton))
  , mCollisionDetector(std::move(_collisionDetector))
  , mCollisionOptions(std::move(_collisionOptions))
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("_metaSkeletonStateSpace is nullptr.");

  // TODO: Check compatibility between MetaSkeleton and MetaSkeletonStateSpace
  if (!mMetaSkeleton)
    throw std::invalid_argument("_metaskeleton is nullptr.");

  if (!mCollisionDetector)
    throw std::invalid_argument("_collisionDetector is nullptr.");
}

//==============================================================================
statespace::ConstStateSpacePtr CollisionFree::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
bool CollisionFree::isSatisfied(
    const aikido::statespace::StateSpace::State* _state,
    TestableOutcome* outcome) const
{
  auto collisionFreeOutcome
      = dynamic_cast_or_throw<CollisionFreeOutcome>(outcome);

  if (collisionFreeOutcome)
  {
    collisionFreeOutcome->clear();
  }

  auto skelStatePtr = static_cast<const aikido::statespace::dart::
                                      MetaSkeletonStateSpace::State*>(_state);
  mMetaSkeletonStateSpace->setState(mMetaSkeleton.get(), skelStatePtr);

  bool collision = false;
  ::dart::collision::CollisionResult collisionResult;
  for (const auto& groups : mGroupsToPairwiseCheck)
  {
    collision = mCollisionDetector->collide(
        groups.first.get(),
        groups.second.get(),
        mCollisionOptions,
        &collisionResult);

    if (collision)
    {
      if (collisionFreeOutcome)
      {
        collisionFreeOutcome->mPairwiseContacts = collisionResult.getContacts();
      }
      return false;
    }
  }

  for (const auto& group : mGroupsToSelfCheck)
  {
    collision = mCollisionDetector->collide(
        group.get(), mCollisionOptions, &collisionResult);
    if (collision)
    {
      if (collisionFreeOutcome)
      {
        collisionFreeOutcome->mSelfContacts = collisionResult.getContacts();
      }
      return false;
    }
  }
  return true;
}

//==============================================================================
std::unique_ptr<TestableOutcome> CollisionFree::createOutcome() const
{
  return std::unique_ptr<TestableOutcome>(new CollisionFreeOutcome);
}

//==============================================================================
void CollisionFree::addPairwiseCheck(
    std::shared_ptr<::dart::collision::CollisionGroup> _group1,
    std::shared_ptr<::dart::collision::CollisionGroup> _group2)
{
  if (_group1 < _group2)
    mGroupsToPairwiseCheck.emplace_back(std::move(_group1), std::move(_group2));
  else
    mGroupsToPairwiseCheck.emplace_back(std::move(_group2), std::move(_group1));
}

//==============================================================================
void CollisionFree::removePairwiseCheck(
    std::shared_ptr<::dart::collision::CollisionGroup> _group1,
    std::shared_ptr<::dart::collision::CollisionGroup> _group2)
{
  if (_group1 < _group2)
    mGroupsToPairwiseCheck.erase(
        std::remove(
            mGroupsToPairwiseCheck.begin(),
            mGroupsToPairwiseCheck.end(),
            std::make_pair(_group1, _group2)),
        mGroupsToPairwiseCheck.end());
  else
    mGroupsToPairwiseCheck.erase(
        std::remove(
            mGroupsToPairwiseCheck.begin(),
            mGroupsToPairwiseCheck.end(),
            std::make_pair(_group2, _group1)),
        mGroupsToPairwiseCheck.end());
}

//==============================================================================
void CollisionFree::addSelfCheck(
    std::shared_ptr<::dart::collision::CollisionGroup> _group)
{
  mGroupsToSelfCheck.emplace_back(std::move(_group));
}

//==============================================================================
void CollisionFree::removeSelfCheck(
    std::shared_ptr<::dart::collision::CollisionGroup> _group)
{
  mGroupsToSelfCheck.erase(
      std::remove(mGroupsToSelfCheck.begin(), mGroupsToSelfCheck.end(), _group),
      mGroupsToSelfCheck.end());
}

//==============================================================================
std::size_t getNodeIndexOf(
    const ::dart::dynamics::BodyNode& bodyNode,
    const ::dart::dynamics::ShapeNode& shapeNode)
{
  for (std::size_t i = 0u; i < bodyNode.getNumShapeNodes(); ++i)
  {
    if (bodyNode.getShapeNode(i) == &shapeNode)
      return i;
  }

  assert(false);
  return 0u;
}

//==============================================================================
::dart::collision::CollisionGroupPtr cloneCollisionGroup(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    const ::dart::dynamics::MetaSkeleton& skeletonClone,
    const ::dart::collision::CollisionGroup& group)
{
  using namespace ::dart::dynamics;
  using namespace ::dart::collision;

  CollisionGroupPtr groupClone
      = collisionDetector->createCollisionGroupAsSharedPtr();

  for (std::size_t i = 0u; i < group.getNumShapeFrames(); ++i)
  {
    const ShapeFrame* shapeFrame = group.getShapeFrame(i);
    const ShapeNode* shapeNode = shapeFrame->asShapeNode();
    assert(shapeNode); // shapeFrame is assumed to be ShapeNode
    assert(shapeNode->getBodyNodePtr());
    const BodyNode* bodyNode = shapeNode->getBodyNodePtr().get();
    assert(skeletonClone.hasBodyNode(bodyNode));
    const BodyNode* bodyNodeClone = skeletonClone.getBodyNode(bodyNode->getName());
    const std::size_t shapeNodeIndex = getNodeIndexOf(*bodyNode, *shapeNode);
    const ShapeNode* shapeNodeClone = bodyNodeClone->getShapeNode(shapeNodeIndex);
    groupClone->addShapeFrame(shapeNodeClone);
  }
  assert(group.getNumShapeFrames() == groupClone->getNumShapeFrames());

  return groupClone;
}

//==============================================================================
TestablePtr CollisionFree::clone(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  using CollisionGroup = ::dart::collision::CollisionGroup;

  // TODO: assert metaSkeleton is a cloned version of mMetaSkeleton
  //
  auto cloned = std::make_shared<CollisionFree>(
      mMetaSkeletonStateSpace,
      metaSkeleton,
      mCollisionDetector,
      mCollisionOptions);

  for (const auto& group : mGroupsToSelfCheck)
  {
    cloned->addSelfCheck(
        cloneCollisionGroup(collisionDetector, *metaSkeleton, *group));
  }

  for (const auto& groupPair : mGroupsToPairwiseCheck)
  {
    cloned->addPairwiseCheck(
        cloneCollisionGroup(collisionDetector, *metaSkeleton, *groupPair.first),
        cloneCollisionGroup(collisionDetector, *metaSkeleton, *groupPair.second));
  }

  return cloned;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
