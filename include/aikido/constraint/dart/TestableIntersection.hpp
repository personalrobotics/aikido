#ifndef AIKIDO_CONSTRAINT_DART_TESTABLEINTERSECTION_HPP_
#define AIKIDO_CONSTRAINT_DART_TESTABLEINTERSECTION_HPP_

#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/constraint/dart/DartConstraint.hpp"

namespace aikido {
namespace constraint {
namespace dart {

AIKIDO_DECLARE_POINTERS(TestableIntersection)

/// A testable constraint grouping a set of testable constraint with DartConstraint.
/// This constriant is satisfied only if all constraints in the set
/// are satisfied.
class TestableIntersection : public constraint::TestableIntersection, public DartConstraint
{
public:
  /// Construct a TestableIntersection on a specific StateSpace.
  /// \param _stateSpace StateSpace this constraint operates in.
  /// \param _constraints Set of constraints.
  TestableIntersection(
      statespace::ConstStateSpacePtr _stateSpace,
      std::vector<ConstTestablePtr> _constraints
      = std::vector<ConstTestablePtr>());

  // Document inherited.
  TestablePtr clone(
      ::dart::collision::CollisionDetectorPtr collisionDetector,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton) const override;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_TESTABLEINTERSECTION_HPP_
