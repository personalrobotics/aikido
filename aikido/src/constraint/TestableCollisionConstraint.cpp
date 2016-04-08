#include <aikido/constraint/TestableCollisionConstraint.hpp>
#include <dart/collision/Result.h>

using namespace aikido::constraint;

bool TestableCollisionConstraint::isSatisfied(
    const aikido::statespace::StateSpace::State* state) const
{
  auto skelStatePtr =
      static_cast<const aikido::statespace::MetaSkeletonStateSpace::State*>(
          state);
  statespace->setStateOnMetaSkeleton(skelStatePtr);

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

void TestableCollisionConstraint::addPairwiseCheck(
    std::shared_ptr<dart::collision::CollisionGroup> group1,
    std::shared_ptr<dart::collision::CollisionGroup> group2)
{
  groupsToPairwiseCheck.push_back(std::make_pair(group1, group2));
}

void TestableCollisionConstraint::addSelfCheck(
    std::shared_ptr<dart::collision::CollisionGroup> group)
{
  groupsToSelfCheck.push_back(group);
}
