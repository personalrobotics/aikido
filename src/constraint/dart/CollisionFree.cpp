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

    std::cout << "Skeleton ids." << std::endl;
    std::cout << "Original :" << skeletonOriginal.getBodyNode(0)->getSkeleton().get() << std::endl;
    std::cout << "Clone    :" << skeletonClone.getBodyNode(0)->getSkeleton().get() << std::endl;
    std::cout << "BodyNode :" << bodyNode->getSkeleton().get() << std::endl;

    if (!skeletonOriginal.hasBodyNode(bodyNode))
    {
      std::cout << "skeletonOriginal [" << skeletonOriginal.getName()
                << "] doesn't contain an instance of bodyNode ["
                << bodyNode->getName() << "].\n";
      if (skeletonOriginal.getBodyNode(bodyNode->getName()))
      {
        std::cout << "  - BodyNode instance in skeletonOriginal: "
                  << skeletonOriginal.getBodyNode(bodyNode->getName())
                  << "\n"
                  << "  - BodyNode instance in skeletonClone   : "
                  << skeletonClone.getBodyNode(bodyNode->getName())
                  << "\n"
                  << "  - bodyNode instance                    : "
                  << bodyNode << "\n";
        std::cout << "BodyNode has parent name " << bodyNode->getSkeleton()->getName() << std::endl;
      }

      groupClone->addShapeFrame(shapeNode);
      continue;
    }

    // GL: shouldn't the clone not have this bodyNode?
    // assert(skeletonClone.hasBodyNode(bodyNode));
    //assert(skeletonOriginal.hasBodyNode(bodyNode)); // GL: Isn't this the check we need?
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
    //std::cout << "shapeNodeIndex " << shapeNodeIndex << std::endl;
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

//==============================================================================
TestablePtr CollisionFree::clone(
    ::dart::collision::CollisionDetectorPtr collisionDetector,
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const
{
  std::cout << "Clone CollisionFree constraint" << std::endl;

  std::cout << "CollisionFree has metaSkeleton " <<
    mMetaSkeleton->getBodyNode(0)->getSkeleton()->getName() << " and uses cloned MetaSkeleton: "
    << metaSkeleton->getBodyNode(0)->getSkeleton()->getName() << std::endl;
  std::cout << "Cloned BodyNode0 " << metaSkeleton->getBodyNode(0)->getName() << std::endl;

  assert(mMetaSkeleton->getBodyNode(0)->getSkeleton() != metaSkeleton->getBodyNode(0)->getSkeleton());

  using CollisionGroup = ::dart::collision::CollisionGroup;

  // TODO: assert metaSkeleton is a cloned version of mMetaSkeleton
  //
  auto cloned = std::make_shared<CollisionFree>(
      mMetaSkeletonStateSpace,
      metaSkeleton,
      mCollisionDetector,
      mCollisionOptions);

  std::cout << "Self check" << std::endl;
  for (const auto& group : mGroupsToSelfCheck)
  {
    cloned->addSelfCheck(
        cloneCollisionGroup(collisionDetector,
          *(mMetaSkeleton->getBodyNode(0)->getSkeleton()),
          *(metaSkeleton->getBodyNode(0)->getSkeleton()), *group));
  }

  std::cout << "Pair check" << std::endl;
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
