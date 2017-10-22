#include <aikido/constraint/NonColliding.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
NonColliding::NonColliding(
    statespace::dart::MetaSkeletonStateSpacePtr _statespace,
    std::shared_ptr<dart::collision::CollisionDetector> _collisionDetector,
    dart::collision::CollisionOption _collisionOptions)
  : statespace(std::move(_statespace))
  , collisionDetector(std::move(_collisionDetector))
  , collisionOptions(std::move(_collisionOptions))
{
  if (!statespace)
    throw std::invalid_argument("_statespace is nullptr.");

  if (!collisionDetector)
    throw std::invalid_argument("_collisionDetector is nullptr.");
}

//==============================================================================
statespace::StateSpacePtr NonColliding::getStateSpace() const
{
  return statespace;
}

//==============================================================================
bool NonColliding::isSatisfied(
    const aikido::statespace::StateSpace::State* _state) const
{
  auto skelStatePtr = static_cast<const aikido::statespace::dart::
                                      MetaSkeletonStateSpace::State*>(_state);
  statespace->setState(skelStatePtr);

  bool collision = false;
  dart::collision::CollisionResult collisionResult;
  for (auto groups : groupsToPairwiseCheck)
  {
    collision = collisionDetector->collide(
        groups.first.get(),
        groups.second.get(),
        collisionOptions,
        &collisionResult);
    if (collision)
      return false;
  }

  for (auto group : groupsToSelfCheck)
  {
    collision = collisionDetector->collide(
        group.get(), collisionOptions, &collisionResult);
    if (collision)
      return false;
  }
  return true;
}

//==============================================================================
void NonColliding::addPairwiseCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group1,
    std::shared_ptr<dart::collision::CollisionGroup> _group2)
{
  groupsToPairwiseCheck.emplace_back(std::move(_group1), std::move(_group2));
}

//==============================================================================
void NonColliding::removePairwiseCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group1,
    std::shared_ptr<dart::collision::CollisionGroup> _group2)
{
  auto it = std::find(
    groupsToPairwiseCheck.begin(), groupsToPairwiseCheck.end(),
    std::make_pair(_group1, _group2));
  if (it != groupsToPairwiseCheck.end())
  {
    groupsToPairwiseCheck.erase(it);
    return;
  }

  // Check for reverse pair
  it = std::find(
    groupsToPairwiseCheck.begin(), groupsToPairwiseCheck.end(),
    std::make_pair(_group2, _group1));
  if (it != groupsToPairwiseCheck.end())
  {
    groupsToPairwiseCheck.erase(it);
    return;
  }
}

//==============================================================================
void NonColliding::addSelfCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group)
{
  groupsToSelfCheck.emplace_back(std::move(_group));
}

//==============================================================================
void NonColliding::removeSelfCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group)
{
  auto it = std::find(
    groupsToSelfCheck.begin(), groupsToSelfCheck.end(),
    _group);
  if (it != groupsToSelfCheck.end())
    groupsToSelfCheck.erase(it);
}

} // namespace constraint
} // namespace aikido
