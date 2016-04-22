#include <aikido/ompl/AIKIDOStateValidityChecker.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

namespace aikido {
namespace ompl {

//=============================================================================
StateValidityChecker::StateValidityChecker(
    const ::ompl::base::SpaceInformationPtr &_si,
    constraint::TestableConstraintPtr _constraint)
    : ::ompl::base::StateValidityChecker(_si)
    , mConstraint(std::move(_constraint))
{
  if (_si == nullptr) {
    throw std::invalid_argument("SpaceInformation is nullptr");
  }

  if (mConstraint == nullptr) {
    throw std::invalid_argument("Constraint is nullptr");
  }
}

//=============================================================================
bool StateValidityChecker::isValid(const ::ompl::base::State *_state) const
{
  auto st = static_cast<const GeometricStateSpace::StateType *>(_state);
  return mConstraint->isSatisfied(st->mState);
}
}
}
