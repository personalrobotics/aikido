#ifndef AIKIDO_CONSTRAINT_DART_COLLISIONFREE_HPP_
#define AIKIDO_CONSTRAINT_DART_COLLISIONFREE_HPP_

#include <memory>
#include <tuple>
#include <vector>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include "aikido/common/pointers.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/dart/CollisionFreeOutcome.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

#include <dart/dart.hpp>

namespace aikido {
namespace constraint {
namespace dart {

AIKIDO_DECLARE_POINTERS(CollisionFree)

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
  /// \param _metaSkeletonStateSpace state space on which the constraint
  /// operates
  /// \param _metaskeleton MetaSkeleton to test with
  /// \param _collisionDetector collision detector used to test for collision
  /// \param _collisionOptions options passed to \c _collisionDetector
  CollisionFree(
      statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
      ::dart::dynamics::MetaSkeletonPtr _metaskeleton,
      std::shared_ptr<::dart::collision::CollisionDetector> _collisionDetector,
      ::dart::collision::CollisionOption _collisionOptions
      = ::dart::collision::CollisionOption(
          false,
          1,
          std::make_shared<::dart::collision::BodyNodeCollisionFilter>()));

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// \copydoc Testable::isSatisfied()
  /// \note Outcome is expected to be an instance of CollisionFreeOutcome.
  /// This method will cast outcome to a pointer of this type, and then populate
  /// the collision information (which bodies are in self/pairwise collision).
  bool isSatisfied(
      const aikido::statespace::StateSpace::State* _state,
      TestableOutcome* outcome = nullptr) const override;

  /// NOTE: Finish partial evaluation.
  bool completeIsSatisfied(
      const aikido::statespace::StateSpace::State* _state,
      std::vector<::dart::collision::narrowPhaseData>& partialRes) const;

  /// \copydoc Testable::createOutcome()
  /// \note Returns an instance of CollisionFreeOutcome.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

  /// Checks collision between group1 and group2.
  /// \param _group1 First collision group.
  /// \param _group2 Second collision group.
  void addPairwiseCheck(
      std::shared_ptr<::dart::collision::CollisionGroup> _group1,
      std::shared_ptr<::dart::collision::CollisionGroup> _group2);

  /// Remove collision check between group1 and group2.
  /// \param _group1 First collision group.
  /// \param _group2 Second collision group.
  void removePairwiseCheck(
      std::shared_ptr<::dart::collision::CollisionGroup> _group1,
      std::shared_ptr<::dart::collision::CollisionGroup> _group2);

  /// Checks collision within group.
  /// \param _group Collision group.
  void addSelfCheck(std::shared_ptr<::dart::collision::CollisionGroup> _group);

  /// Remove self-collision check within group.
  /// \param _group Collision group.
  void removeSelfCheck(
      std::shared_ptr<::dart::collision::CollisionGroup> _group);

private:
  using CollisionGroup = ::dart::collision::CollisionGroup;

  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
      mMetaSkeletonStateSpace;
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  std::shared_ptr<::dart::collision::CollisionDetector> mCollisionDetector;
  ::dart::collision::CollisionOption mCollisionOptions;
  std::vector<std::pair<std::shared_ptr<CollisionGroup>,
                        std::shared_ptr<CollisionGroup>>>
      mGroupsToPairwiseCheck;
  std::vector<std::shared_ptr<CollisionGroup>> mGroupsToSelfCheck;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_COLLISIONFREE_HPP_
