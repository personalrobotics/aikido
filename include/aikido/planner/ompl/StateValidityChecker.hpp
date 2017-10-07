#ifndef AIKIDO_PLANNER_OMPL_AIKIDOSTATEVALIDITYCHECKER_HPP_
#define AIKIDO_PLANNER_OMPL_AIKIDOSTATEVALIDITYCHECKER_HPP_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include "../../constraint/Testable.hpp"
#include "../../statespace/StateSpace.hpp"

namespace aikido {
namespace planner {
namespace ompl {

/// Expose a set of aikido::constraint::Testable class as a
/// StateValidityChecker to the OMPL framework.  This checker will mark a state
/// valid if all constraints defined within the class are valid for the state.
class StateValidityChecker : public ::ompl::base::StateValidityChecker
{

public:
  /// Constructor
  /// \param _si Information about the planning space where this ValidityChecker
  /// will be used
  /// \param _constraint The constraint that must pass for a state to
  /// be marked valid
  StateValidityChecker(
      const ::ompl::base::SpaceInformationPtr& _si,
      constraint::TestablePtr _constraint);

  /// Return true if all constraints defined on this ValidityChecker are
  /// satisfied.
  /// \param _state The state to check
  bool isValid(const ::ompl::base::State* _state) const override;

private:
  constraint::TestablePtr mConstraint;
};

} // namespace ompl
} // namespace planner
} // namespace aikido

#endif
