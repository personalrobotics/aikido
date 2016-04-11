#ifndef AIKIDO_CONSTRAINT_COLLISION_CONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_COLLISION_CONSTRAINT_HPP_

#include "TestableConstraint.hpp"
#include <vector>
#include <tuple>
#include "../statespace/MetaSkeletonStateSpace.hpp"
#include <dart/collision/CollisionDetector.h>
#include <dart/collision/Option.h>
#include <dart/collision/CollisionGroup.h>

using std::shared_ptr;

namespace aikido
{
namespace constraint
{
class CollisionConstraint : public TestableConstraint
{
public:
  CollisionConstraint(
      std::shared_ptr<aikido::statespace::MetaSkeletonStateSpace> statespace,
      std::shared_ptr<dart::collision::CollisionDetector> collisionDetector)
      : statespace{statespace}
      , collisionDetector{collisionDetector}
      , collisionOptions{false, true}
  {
  }

  const shared_ptr<aikido::statespace::StateSpace> getStateSpace() const override
  {
    return statespace;
  }

  bool isSatisfied(const aikido::statespace::StateSpace::State* state) const override;

  void addPairwiseCheck(
      std::shared_ptr<dart::collision::CollisionGroup> group1,
      std::shared_ptr<dart::collision::CollisionGroup> group2);

  void addSelfCheck(
      std::shared_ptr<dart::collision::CollisionGroup> group);

  // void setCollisionMask(std::function_bool) // to collisionfilter -> option

private:
  using CollisionGroup = dart::collision::CollisionGroup;

  shared_ptr<aikido::statespace::MetaSkeletonStateSpace> statespace;
  shared_ptr<dart::collision::CollisionDetector> collisionDetector;
  dart::collision::CollisionOption collisionOptions;
  std::vector<std::pair<shared_ptr<CollisionGroup>, shared_ptr<CollisionGroup>>>
      groupsToPairwiseCheck;
  std::vector<shared_ptr<CollisionGroup>> groupsToSelfCheck;
};

}  // constraint
}  // aikido
#endif  // AIKIDO_CONSTRAINT_COLLISION_CONSTRAINT_HPP_
