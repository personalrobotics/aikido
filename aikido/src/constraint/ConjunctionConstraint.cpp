#include <aikido/constraint/ConjunctionConstraint.hpp>
#include <stdexcept>

namespace aikido {
namespace constraint {

//=============================================================================
ConjunctionConstraint::ConjunctionConstraint(
    statespace::StateSpacePtr _stateSpace,
    std::vector<std::shared_ptr<TestableConstraint>> _constraints)
: mStateSpace(std::move(_stateSpace))
, mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("_statespace is nullptr.");

  for (auto c : mConstraints) {
    testConstraintStateSpaceOrThrow(c);
  }
}

//=============================================================================
bool ConjunctionConstraint::isSatisfied(
    const aikido::statespace::StateSpace::State* _state) const
{
  for (auto c : mConstraints) {
    if (!c->isSatisfied(_state)) {
      return false;
    }
  }
  return true;
}

//=============================================================================
statespace::StateSpacePtr ConjunctionConstraint::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
void ConjunctionConstraint::addConstraint(TestableConstraintPtr _constraint)
{
  if (_constraint->getStateSpace() == mStateSpace) {
    mConstraints.emplace_back(std::move(_constraint));
  } else {
    throw std::invalid_argument{
        "Constraints must all be in specified StateSpace"};
  }
}

//=============================================================================
void ConjunctionConstraint::testConstraintStateSpaceOrThrow(
  const TestableConstraintPtr& constraint)
{
  if (constraint->getStateSpace() != mStateSpace) {
    throw std::invalid_argument{
        "Constraints must all be in specified StateSpace"};
  }
}

}
}
