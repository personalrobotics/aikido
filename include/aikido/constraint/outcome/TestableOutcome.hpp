#ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_

#include <string>

namespace aikido {
namespace constraint {

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

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_TESTABLEOUTCOME_HPP_
