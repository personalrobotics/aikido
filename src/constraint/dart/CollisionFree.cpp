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

  auto robot = mMetaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  auto skelStatePtr = static_cast<
      const aikido::statespace::dart::MetaSkeletonStateSpace::State*>(_state);
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

} // namespace dart
} // namespace constraint
} // namespace aikido
