#include <aikido/constraint/CollisionFree.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
CollisionFree::CollisionFree(
    statespace::dart::MetaSkeletonStateSpacePtr _statespace,
    std::shared_ptr<dart::collision::CollisionDetector> _collisionDetector,
    dart::collision::CollisionOption _collisionOptions)
  : mStatespace(std::move(_statespace))
  , mCollisionDetector(std::move(_collisionDetector))
  , mCollisionOptions(std::move(_collisionOptions))
{
  if (!mStatespace)
    throw std::invalid_argument("_statespace is nullptr.");

  if (!mCollisionDetector)
    throw std::invalid_argument("_collisionDetector is nullptr.");
}

//==============================================================================
statespace::StateSpacePtr CollisionFree::getStateSpace() const
{
  return mStatespace;
}

//==============================================================================
bool CollisionFree::isSatisfied(
    const aikido::statespace::StateSpace::State* _state) const
{
  auto skelStatePtr = static_cast<const aikido::statespace::dart::
                                      MetaSkeletonStateSpace::State*>(_state);
  mStatespace->setState(skelStatePtr);

  bool collision = false;
  dart::collision::CollisionResult collisionResult;
  for (auto groups : mGroupsToPairwiseCheck)
  {
    collision = mCollisionDetector->collide(
        groups.first.get(),
        groups.second.get(),
        mCollisionOptions,
        &collisionResult);
    if (collision)
      return false;
  }

  for (auto group : mGroupsToSelfCheck)
  {
    collision = mCollisionDetector->collide(
        group.get(), mCollisionOptions, &collisionResult);
    if (collision)
      return false;
  }
  return true;
}

//==============================================================================
void CollisionFree::addPairwiseCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group1,
    std::shared_ptr<dart::collision::CollisionGroup> _group2)
{
  if (_group1 < _group2)
    mGroupsToPairwiseCheck.emplace_back(std::move(_group1), std::move(_group2));
  else
    mGroupsToPairwiseCheck.emplace_back(std::move(_group2), std::move(_group1));
}

//==============================================================================
void CollisionFree::removePairwiseCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group1,
    std::shared_ptr<dart::collision::CollisionGroup> _group2)
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
    std::shared_ptr<dart::collision::CollisionGroup> _group)
{
  mGroupsToSelfCheck.emplace_back(std::move(_group));
}

//==============================================================================
void CollisionFree::removeSelfCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group)
{
  mGroupsToSelfCheck.erase(
      std::remove(mGroupsToSelfCheck.begin(), mGroupsToSelfCheck.end(), _group),
      mGroupsToSelfCheck.end());
}

} // namespace constraint
} // namespace aikido
