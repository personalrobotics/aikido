#ifndef AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_

#include <memory>
#include "../statespace/StateSpace.hpp"

namespace aikido {
namespace constraint {

/// Constraint which can be tested.
class TestableConstraint
{
public:
  virtual ~TestableConstraint() = default;

  /// Returns true if state satisfies this constraint.
  virtual bool isSatisfied(
    const statespace::StateSpace::State* _state) const = 0;

  /// Retursn StateSpace in which this constraint operates.
  virtual statespace::StateSpacePtr getStateSpace() const = 0;
};

using TestableConstraintPtr = std::shared_ptr<TestableConstraint>;

}  // constraint
}  // aikido

#endif  // AIKIDO_CONSTRAINT_TESTABLE_CONSTRAINT_HPP_
