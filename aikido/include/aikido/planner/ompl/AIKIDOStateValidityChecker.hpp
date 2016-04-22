#ifndef AIKIDO_OMPL_AIKIDOSTATEVALIDITYCHECKER_HPP_
#define AIKIDO_OMPL_AIKIDOSTATEVALIDITYCHECKER_HPP_

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include "../../statespace/StateSpace.hpp"
#include "../../constraint/TestableConstraint.hpp"

namespace aikido {
namespace ompl {
/// Expose a set of aikido::conststraint::TestableConstraint class as a
/// StateValidityChecker to the OMPL framework.  This checker will mark a state
/// valid if all constraints defined within the class are valid for the state.
class StateValidityChecker : public ::ompl::base::StateValidityChecker {

public:
  /// Constructor
  /// \param _si Information about the planning space where this ValidityChecker
  /// will be used
  /// \param _constraint The constraint that must pass for a state to
  /// be marked valid
  StateValidityChecker(
      const ::ompl::base::SpaceInformationPtr &_si,
      constraint::TestableConstraintPtr _constraint);

  /// Return true if all constraints defined on this ValidityChecker are satisfied.
  /// \param _state The state to check
  bool isValid(const ::ompl::base::State *_state) const override;

private:
  constraint::TestableConstraintPtr mConstraint;

};
}
}
#endif
