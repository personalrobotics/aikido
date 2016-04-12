#include <aikido/ompl/AIKIDOStateValidityChecker.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

namespace aikido {
namespace ompl {

AIKIDOStateValidityChecker::AIKIDOStateValidityChecker(
    const ::ompl::base::SpaceInformationPtr &_si,
    std::vector<std::shared_ptr<aikido::constraint::TestableConstraint>>
        _constraints)
    : ::ompl::base::StateValidityChecker(_si),
      mConstraints(std::move(_constraints)) {}

bool AIKIDOStateValidityChecker::isValid(
    const ::ompl::base::State *_state) const {

  // Check constraints
  auto state =
      static_cast<const AIKIDOGeometricStateSpace::StateType *>(_state);
  for (auto &constraint : mConstraints) {
    if (!constraint->isSatisfied(state->mState)) {
      return false;
    }
  }

  return true;
}
}
}
