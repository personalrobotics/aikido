#include <aikido/constraint/CollisionConstraint.hpp>
#include <dart/collision/Result.h>

using namespace aikido::constraint;

bool CollisionConstraint::isSatisfied(
    const aikido::statespace::StateSpace::State* state) const
{
  auto skelStatePtr =
      static_cast<const aikido::statespace::MetaSkeletonStateSpace::State*>(
          state);
  statespace->setStateOnMetaSkeleton(skelStatePtr);

  bool collision = false;
  dart::collision::CollisionResult collisionResult;
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

void CollisionConstraint::addPairwiseCheck(
    std::shared_ptr<dart::collision::CollisionGroup> group1,
    std::shared_ptr<dart::collision::CollisionGroup> group2)
{
  groupsToPairwiseCheck.push_back(std::make_pair(group1, group2));
}

void CollisionConstraint::addSelfCheck(
    std::shared_ptr<dart::collision::CollisionGroup> group)
{
  groupsToSelfCheck.push_back(group);
}
