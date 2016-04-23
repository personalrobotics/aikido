#ifndef AIKIDO_CONSTRAINT_NONCOLLIDING_HPP_
#define AIKIDO_CONSTRAINT_NONCOLLIDING_HPP_

#include "Testable.hpp"
#include <vector>
#include <tuple>
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include <dart/collision/CollisionDetector.h>
#include <dart/collision/Option.h>
#include <dart/collision/CollisionGroup.h>

namespace aikido {
namespace constraint {

/// A testable that uses a collision detector to check whether 
/// a metakeleton state (configuration) results in collision between and within
/// specified collision groups.
class NonColliding : public Testable
{
public:
  NonColliding(
      statespace::dart::MetaSkeletonStateSpacePtr _statespace,
      std::shared_ptr<dart::collision::CollisionDetector> _collisionDetector);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool isSatisfied(
    const aikido::statespace::StateSpace::State* _state) const override;

  /// Checks collision between group1 and group2.
  /// \param group1 First collision group.
  /// \param group2 Second collision group.
  void addPairwiseCheck(
      std::shared_ptr<dart::collision::CollisionGroup> _group1,
      std::shared_ptr<dart::collision::CollisionGroup> _group2);

  /// Checks collision within group.
  /// \param group Collision group.
  void addSelfCheck(
      std::shared_ptr<dart::collision::CollisionGroup> _group);

  // void setCollisionMask(std::function_bool) // to collisionfilter -> option

private:
  using CollisionGroup = dart::collision::CollisionGroup;

  std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> statespace;
  std::shared_ptr<dart::collision::CollisionDetector> collisionDetector;
  dart::collision::Option collisionOptions;
  std::vector<std::pair<std::shared_ptr<CollisionGroup>, std::shared_ptr<CollisionGroup>>>
      groupsToPairwiseCheck;
  std::vector<std::shared_ptr<CollisionGroup>> groupsToSelfCheck;
};

}  // constraint
}  // aikido
#endif  // AIKIDO_CONSTRAINT_NONCOLLIDING_HPP_
