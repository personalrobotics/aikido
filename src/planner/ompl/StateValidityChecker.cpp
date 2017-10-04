#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/planner/ompl/StateValidityChecker.hpp>

namespace aikido {
namespace planner {
namespace ompl {

//==============================================================================
StateValidityChecker::StateValidityChecker(
    const ::ompl::base::SpaceInformationPtr& _si,
    constraint::TestablePtr _constraint)
  : ::ompl::base::StateValidityChecker(_si), mConstraint(std::move(_constraint))
{
  if (_si == nullptr)
  {
    throw std::invalid_argument("SpaceInformation is nullptr");
  }

  if (mConstraint == nullptr)
  {
    throw std::invalid_argument("Constraint is nullptr");
  }
}

//==============================================================================
bool StateValidityChecker::isValid(const ::ompl::base::State* _state) const
{
  auto st = static_cast<const GeometricStateSpace::StateType*>(_state);
  if (st == nullptr || st->mState == nullptr)
    return false;

  if (!st->mValid)
    return false;

  return mConstraint->isSatisfied(st->mState);
}

} // namespace ompl
} // namespace planner
} // namespace aikido
