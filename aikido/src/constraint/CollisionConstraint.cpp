#include <aikido/constraint/CollisionConstraint.hpp>
#include <dart/collision/Result.h>

namespace aikido {
namespace constraint {

//=============================================================================
CollisionConstraint::CollisionConstraint(
    statespace::dart::MetaSkeletonStateSpacePtr _statespace,
    std::shared_ptr<dart::collision::CollisionDetector> _collisionDetector)
: statespace(std::move(_statespace))
, collisionDetector(std::move(_collisionDetector))
, collisionOptions{false, true}
{
  if (!statespace)
    throw std::invalid_argument("_statespace is nullptr.");

  if (!collisionDetector)
    throw std::invalid_argument("_collisionDetector is nullptr.");

}

//=============================================================================
statespace::StateSpacePtr CollisionConstraint::getStateSpace() const
{
  return statespace;
}

//=============================================================================
bool CollisionConstraint::isSatisfied(
    const aikido::statespace::StateSpace::State* _state) const
{
  auto skelStatePtr =
    static_cast<const aikido::statespace::dart::MetaSkeletonStateSpace::State*>(
      _state);
  statespace->setState(skelStatePtr);

  bool collision = false;
  dart::collision::Result collisionResult;
  for (auto groups : groupsToPairwiseCheck) {
    collision =
        collisionDetector->collide(groups.first.get(), groups.second.get(),
                                   collisionOptions, collisionResult);
    if (collision) return false;
  }

  for (auto group : groupsToSelfCheck) {
    collision = collisionDetector->collide(group.get(), collisionOptions,
                                           collisionResult);
    if (collision) return false;
  }
  return true;
}

//=============================================================================
void CollisionConstraint::addPairwiseCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group1,
    std::shared_ptr<dart::collision::CollisionGroup> _group2)
{
  groupsToPairwiseCheck.emplace_back(std::move(_group1), std::move(_group2));
}

//=============================================================================
void CollisionConstraint::addSelfCheck(
    std::shared_ptr<dart::collision::CollisionGroup> _group)
{
  groupsToSelfCheck.emplace_back(std::move(_group));
}

}
}
