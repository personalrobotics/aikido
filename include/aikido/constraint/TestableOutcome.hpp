#ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_

#include <sstream>
#include <stdexcept>
#include <typeinfo>

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
Child* dynamic_cast_if_present(TestableOutcome* _outcome)
{
  if (!_outcome)
    return nullptr;

  auto childPtr = dynamic_cast<Child*>(_outcome);
  if (!childPtr)
  {
    std::stringstream message;
    message << "TestableOutcome pointer is not of type " << typeid(Child).name()
            << ".";
    throw std::invalid_argument(message.str());
  }

  return childPtr;
}

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
