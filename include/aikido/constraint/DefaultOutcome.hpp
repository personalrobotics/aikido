#ifndef AIKIDO_CONSTRAINT_DUMMYOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_DUMMYOUTCOME_HPP_

#include "TestableOutcome.hpp"

namespace aikido {
namespace constraint {

/// Simple default TestableOutcome derivative class. An instance of this class
/// is returned when createOutcome() is called on an instance of a class that
/// inherits Testable, but has no corresponding TestableOutcome derivative
/// implemented.
class DefaultOutcome : public TestableOutcome
{
public:
  /// Nonsense.
  bool isSatisfied() const override
  {
    throw std::runtime_error(
        "This is a dummy constraint outcome. The Testable derivative class you "
        "called createOutcome() on has no corresponding TestableOutcome "
        "derivative implemented. Do not call isSatisfied.");
  }

  /// Nonsense.
  std::string toString() const override
  {
    return "This is a dummy constraint outcome. The Testable derivative class "
           "you called createOutcome() on has no corresponding TestableOutcome "
           "derivative implemented.";
  }
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DUMMYOUTCOME_HPP_
