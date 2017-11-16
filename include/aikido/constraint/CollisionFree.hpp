#ifndef AIKIDO_CONSTRAINT_COLLISIONFREE_HPP_
#define AIKIDO_CONSTRAINT_COLLISIONFREE_HPP_

#include <memory>
#include <tuple>
#include <vector>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include "Testable.hpp"

namespace aikido {
namespace constraint {

/// A testable that uses a collision detector to check whether
/// a metakeleton state (configuration) results in collision between and within
/// specified collision groups.
class CollisionFree : public Testable
{
public:
  /// Constructs an empty constraint that uses \c _collisionDetector to test
  /// for collision. You should call \c addPairWiseCheck and \c addSelfCheck
  /// to register collision checks before calling \c isSatisfied.
  ///
  /// \param _statespace state space on which the constraint operates
  /// \param _collisionDetector collision detector used to test for collision
  /// \param _collisionOptions options passed to \c _collisionDetector
  CollisionFree(
      statespace::dart::MetaSkeletonStateSpacePtr _statespace,
      std::shared_ptr<dart::collision::CollisionDetector> _collisionDetector,
      dart::collision::CollisionOption _collisionOptions
      = dart::collision::CollisionOption(
          false,
          1,
          std::make_shared<dart::collision::BodyNodeCollisionFilter>()));

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
  void addSelfCheck(std::shared_ptr<dart::collision::CollisionGroup> _group);

private:
  using CollisionGroup = dart::collision::CollisionGroup;

  std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace> statespace;
  std::shared_ptr<dart::collision::CollisionDetector> collisionDetector;
  dart::collision::CollisionOption collisionOptions;
  std::vector<std::pair<std::shared_ptr<CollisionGroup>,
                        std::shared_ptr<CollisionGroup>>>
      groupsToPairwiseCheck;
  std::vector<std::shared_ptr<CollisionGroup>> groupsToSelfCheck;
};

using CollisionFreePtr = std::shared_ptr<CollisionFree>;

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_COLLISIONFREE_HPP_
