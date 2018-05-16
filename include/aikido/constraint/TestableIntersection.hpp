#ifndef AIKIDO_CONSTRAINT_TESTABLEINTERSECTION_HPP_
#define AIKIDO_CONSTRAINT_TESTABLEINTERSECTION_HPP_

#include <memory>
#include <vector>
#include "Testable.hpp"

namespace aikido {
namespace constraint {

AIKIDO_DECLARE_POINTERS(TestableIntersection)

/// A testable constraint grouping a set of testable constraint.
/// This constriant is satisfied only if all constraints in the set
/// are satisfied.
class TestableIntersection : public Testable
{
public:
  /// Construct a TestableIntersection on a specific StateSpace.
  /// \param _stateSpace StateSpace this constraint operates in.
  /// \param _constraints Set of constraints.
  TestableIntersection(
      statespace::StateSpacePtr _stateSpace,
      std::vector<ConstTestablePtr> _constraints = std::vector<ConstTestablePtr>());

  // Documentation inherited.
  bool isSatisfied(
      const aikido::statespace::StateSpace::State* state,
      TestableOutcome* outcome = nullptr) const override;

  /// Return an instance of DefaultTestableOutcome, since this class doesn't
  /// have a more specialized TestableOutcome derivative assigned to it.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Add a Testable to the conjunction.
  /// \param constraint a constraint in the same StateSpace as the
  ///        TestableIntersection was initialize with.
  void addConstraint(ConstTestablePtr constraint);

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<ConstTestablePtr> mConstraints;

  void testConstraintStateSpaceOrThrow(const ConstTestablePtr& constraint);
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_TESTABLEINTERSECTION_HPP_
