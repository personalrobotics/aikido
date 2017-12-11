#ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_

#include <string>

namespace aikido {
namespace constraint {

/// Base class for constraint outcomes. At a high level, each constraint can
/// have a corresponding derivative of TestableOutcome that is passed as an
/// optional parameter to its isSatisfied method. This allow programmatic access
/// to data on why the constraint was (or was not) satisfied.
class TestableOutcome
{
public:
  /// Returns true if isSatisfied call this outcome object was passed to
  /// returned true. False otherwise.
  virtual bool isSatisfied() const = 0;

  /// String representation of the outcome. Provides useful, programmatic
  /// access to debug information.
  virtual std::string toString() const = 0;
};

/// Helper function. Avoids repeating logic for casting TestableOutcome
/// pointers down to pointers for a derivative class. Mostly used in the
/// isSatisfied methods of classes that inherit Testable.
template <class Child>
Child* dynamic_cast_if_present(TestableOutcome* outcome);

} // namespace constraint
} // namespace aikido

#include "detail/TestableOutcome-impl.hpp"

#endif // AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
