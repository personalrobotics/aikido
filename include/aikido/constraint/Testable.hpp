#ifndef AIKIDO_CONSTRAINT_TESTABLE_HPP_
#define AIKIDO_CONSTRAINT_TESTABLE_HPP_

#include <memory>
#include "../statespace/StateSpace.hpp"

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
};

using TestablePtr = std::shared_ptr<Testable>;

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_TESTABLE_HPP_
