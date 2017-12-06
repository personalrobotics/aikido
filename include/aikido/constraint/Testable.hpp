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
  /// \param[in] _state given state to test.
  /// \param[in] outcome pointer to TestableOutcome derivative instance that
  /// method will populate with useful information. Each derivative class of 
  /// Testable may expect outcome to be a different derivative class of
  /// TestableOutcome (this casting and population is done under the hood). If
  /// this argument is missing, it is ignored.
  virtual bool isSatisfied(
      const statespace::StateSpace::State* _state,
      TestableOutcome* outcome = nullptr) const = 0;

  /// Returns StateSpace in which this constraint operates.
  virtual statespace::StateSpacePtr getStateSpace() const = 0;

  /// Return an instance of a TestableOutcome derivative class that corresponds
  /// to this constraint class. Ensures that correct outcome object is passed
  /// to isSatisfied (and casts, etc do not explode).
  virtual std::unique_ptr<TestableOutcome> createOutcome() const = 0;
};

using TestablePtr = std::shared_ptr<Testable>;

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_TESTABLE_HPP_