#include "aikido/constraint/dart/CollisionFree.hpp"

#include <dart/collision/collision.hpp>

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

  std::cout << "[CollisionFree] Error: Failed to find shapeNode ["
            << shapeNode.getName() << "] from bodyNode ["
            << bodyNode.getName() << "].\n";
  exit(1);
  return 0u;
}

//==============================================================================
::dart::collision::CollisionGroupPtr cloneCollisionGroup(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    const ::dart::dynamics::MetaSkeleton& skeletonOriginal,
    const ::dart::dynamics::MetaSkeleton& skeletonClone,
    const ::dart::collision::CollisionGroup& group)
{
  using namespace ::dart::dynamics;
  using namespace ::dart::collision;

  CollisionGroupPtr groupClone
      = collisionDetector->createCollisionGroupAsSharedPtr();

  if (skeletonOriginal.getName() != skeletonClone.getName())
  {
    std::cout << "skeletonOriginal and skeletonClone have different names.\n";
    exit(1);
  }

  for (std::size_t i = 0u; i < group.getNumShapeFrames(); ++i)
  {
    const ShapeFrame* shapeFrame = group.getShapeFrame(i);
    const ShapeNode* shapeNode = shapeFrame->asShapeNode();
    assert(shapeNode); // shapeFrame is assumed to be ShapeNode
    assert(shapeNode->getBodyNodePtr());
    const BodyNode* bodyNode = shapeNode->getBodyNodePtr().get();

    // If bodyNode is not in the original skeleton, we assume the object is
    // meant to be shared.
    if (!skeletonOriginal.hasBodyNode(bodyNode))
    {
      groupClone->addShapeFrame(shapeNode);
      continue;
    }

    assert(skeletonOriginal.hasBodyNode(bodyNode) || skeletonClone.hasBodyNode(bodyNode)
        || skeletonClone.getBodyNode(bodyNode->getName()));

    const BodyNode* bodyNodeClone = skeletonClone.getBodyNode(bodyNode->getName());

    if (bodyNode->getNumShapeNodes() != bodyNodeClone->getNumShapeNodes())
    {
      std::cout << "In skeletonOriginal [" << skeletonOriginal.getName()
                << "]:\n";
      std::cout << "BodyNode      [" << bodyNode->getName()
        << "] has " << bodyNode->getNumShapeNodes() << " shapeNodes" << std::endl;
      std::cout << "BodyNodeClone [" << bodyNodeClone->getName()
        << "] has " << bodyNodeClone->getNumShapeNodes() << " shapeNodes" << std::endl;
      exit(1);
    }

    const std::size_t shapeNodeIndex = getNodeIndexOf(*bodyNode, *shapeNode);
    const ShapeNode* shapeNodeClone = bodyNodeClone->getShapeNode(shapeNodeIndex);
    assert(bodyNodeClone);
    assert(shapeNodeClone);

    groupClone->addShapeFrame(shapeNodeClone);
  }

  if (group.getNumShapeFrames() != groupClone->getNumShapeFrames())
  {
    std::cout << "Num shapes " << group.getNumShapeFrames()
      << " vs " << groupClone->getNumShapeFrames() << std::endl;
    exit(1);
  }

  return groupClone;
}

//=============================================================================
const ::dart::dynamics::BodyNode* getBodyNode(
    const ::dart::dynamics::BodyNode* bodyNode,
    const ::dart::dynamics::MetaSkeletonPtr& metaSkeleton)
{
  const ::dart::dynamics::BodyNode* bodyNodeClone
      = metaSkeleton->getBodyNode(0)->getSkeleton()->getBodyNode(bodyNode->getName());

  // Assume bodyNode is meant to be cloned but a bodyNode shared over envs
  if (!bodyNodeClone)
    return bodyNode;

  return bodyNodeClone;
}

//==============================================================================
TestablePtr CollisionFree::clone(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  using namespace ::dart::collision;
  using namespace ::dart::dynamics;

  if (mMetaSkeleton->getBodyNode(0)->getSkeleton()
      == metaSkeleton->getBodyNode(0)->getSkeleton())
  {
    std::stringstream ss;
    ss << "CollisionFree has metaSkeleton "
              << mMetaSkeleton->getBodyNode(0)->getSkeleton()->getName()
              << " and uses cloned MetaSkeleton: "
              << metaSkeleton->getBodyNode(0)->getSkeleton()->getName()
              << std::endl
              << "Cloned BodyNode0 " << metaSkeleton->getBodyNode(0)->getName()
              << std::endl;
    throw std::runtime_error(ss.str());
  }

  auto collOptionClone = mCollisionOptions;
  auto bodyNodeCollisionFilter
      = std::dynamic_pointer_cast<BodyNodeCollisionFilter>(
        mCollisionOptions.collisionFilter);
  if (bodyNodeCollisionFilter)
  {
    auto newCollisionFilter = std::make_shared<BodyNodeCollisionFilter>();
    const ::dart::collision::detail::UnorderedPairs<BodyNode>& pairs
        = bodyNodeCollisionFilter->getPairs();
    const auto& map = pairs.getMap();
    for (const auto& leftAndRights : map)
    {
      const auto& left = leftAndRights.first;
      const auto& rights = leftAndRights.second;

      for (const auto& right : rights)
      {
        newCollisionFilter->addBodyNodePairToBlackList(
            getBodyNode(left, metaSkeleton),
            getBodyNode(right, metaSkeleton));
      }
    }
    collOptionClone.collisionFilter = newCollisionFilter;
  }
  else
  {
    collOptionClone.collisionFilter = mCollisionOptions.collisionFilter;
  }

  auto cloned = std::make_shared<CollisionFree>(
      mMetaSkeletonStateSpace,
      metaSkeleton,
      collisionDetector,
      collOptionClone);

  for (const auto& group : mGroupsToSelfCheck)
  {
    cloned->addSelfCheck(
        cloneCollisionGroup(collisionDetector,
          *(mMetaSkeleton->getBodyNode(0)->getSkeleton()),
          *(metaSkeleton->getBodyNode(0)->getSkeleton()), *group));
  }

  for (const auto& groupPair : mGroupsToPairwiseCheck)
  {
    cloned->addPairwiseCheck(
        cloneCollisionGroup(collisionDetector,
          *(mMetaSkeleton->getBodyNode(0)->getSkeleton()),
          *(metaSkeleton->getBodyNode(0)->getSkeleton()), *groupPair.first),
        cloneCollisionGroup(collisionDetector,
          *(mMetaSkeleton->getBodyNode(0)->getSkeleton()),
          *(metaSkeleton->getBodyNode(0)->getSkeleton()), *groupPair.second));
  }

  return cloned;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
