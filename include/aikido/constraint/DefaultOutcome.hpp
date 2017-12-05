#ifndef AIKIDO_CONSTRAINT_DEFAULTOUTCOME_HPP_
#define AIKIDO_CONSTRAINT_DEFAULTOUTCOME_HPP_

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
  /// Returns whether the isSatisfied method this object was passed to
  /// returned true or false.
  bool isSatisfied() const override;

  /// String representation of isSatisfied return value.
  std::string toString() const override;

  /// Used by the isSatisfied this outcome object is passed to set whether the
  /// constraint was satisifed or not.
  /// \param[in] satisfiedFlag whether the constraint was satisfied or not.
  void setSatisfiedFlag(bool satisfiedFlag);


protected:
  bool mSatisfiedFlag;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DEFAULTOUTCOME_HPP_
