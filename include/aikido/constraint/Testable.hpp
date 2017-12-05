#ifndef AIKIDO_CONSTRAINT_TESTABLE_HPP_
#define AIKIDO_CONSTRAINT_TESTABLE_HPP_

#include <memory>
#include "../statespace/StateSpace.hpp"
#include "DefaultOutcome.hpp"

namespace aikido {
namespace constraint {

class TestableOutcome;

/// Constraint which can be tested.
class Testable
{
public:
  virtual ~Testable() = default;

  /// Returns true if state satisfies this constraint.
  virtual bool isSatisfied(
      const statespace::StateSpace::State* _state,
      TestableOutcome* outcome = nullptr) const = 0;

  /// Returns StateSpace in which this constraint operates.
  virtual statespace::StateSpacePtr getStateSpace() const = 0;

  /// Return an instance of a TestableOutcome derivative class that corresponds
  /// to this constraint class. Ensures that correct outcome object is passed
  /// to isSatisfied (and casts, etc do not explode). Default implementation
  /// just returns a dummy object.
  virtual std::unique_ptr<TestableOutcome> createOutcome() const
  {
    return std::unique_ptr<TestableOutcome>(new DefaultOutcome());
  }
};

using TestablePtr = std::shared_ptr<Testable>;

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_TESTABLE_HPP_