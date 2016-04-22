#ifndef AIKIDO_CONSTRAINT_CONJUNCTION_HPP_
#define AIKIDO_CONSTRAINT_CONJUNCTION_HPP_

#include "Testable.hpp"
#include <memory>
#include <vector>

namespace aikido {
namespace constraint {

/// A testable constraint grouping a set of testable constraint.
/// This constriant is satisfied only if all constraints in the set
/// are satisfied.
class ConjunctionConstraint : public Testable
{
public:
  /// Construct a ConjunctionConstraint on a specific StateSpace.
  /// \param statespace StateSpace this constraint operates in.
  /// \param constraints Set of constraints.
  ConjunctionConstraint(
    statespace::StateSpacePtr _stateSpace,
    std::vector<TestablePtr> _constraints = 
      std::vector<TestablePtr>());

  // Documentation inherited.
  bool isSatisfied(
    const aikido::statespace::StateSpace::State* state) const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Add a Testable to the conjunction.
  /// \param constraint a constraint in the same StateSpace as the
  ///        ConjunctionConstraint was initialize with.
  void addConstraint(TestablePtr constraint);

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<TestablePtr> mConstraints;

  void testConstraintStateSpaceOrThrow(const TestablePtr& constraint);
};
}  // constraint
}  // aikido

#endif  // AIKIDO_CONSTRAINT_CONJUNCTION_HPP_
