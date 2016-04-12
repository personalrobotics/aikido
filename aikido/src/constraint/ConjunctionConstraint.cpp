#include <aikido/constraint/ConjunctionConstraint.hpp>
#include <stdexcept>

using namespace aikido::constraint;

ConjunctionConstraint::ConjunctionConstraint(
    std::shared_ptr<aikido::statespace::StateSpace> stateSpace,
    std::vector<std::shared_ptr<TestableConstraint>> constraints)
    : mStateSpace(stateSpace)
    , mConstraints{constraints}
{
  for (auto c : mConstraints) {
    testConstraintStateSpaceOrThrow(c);
  }
}

bool ConjunctionConstraint::isSatisfied(
    const aikido::statespace::StateSpace::State* state) const
{
  for (auto c : mConstraints) {
    if (!c->isSatisfied(state)) {
      return false;
    }
  }
  return true;
}

const std::shared_ptr<aikido::statespace::StateSpace>
ConjunctionConstraint::getStateSpace() const
{
  return mStateSpace;
}

void ConjunctionConstraint::addConstraint(
    std::shared_ptr<TestableConstraint> constraint)
{
  if (constraint->getStateSpace() == mStateSpace) {
    mConstraints.push_back(constraint);
  } else {
    throw std::invalid_argument{
        "Constriants must all be in specified StateSpace"};
  }
}

void ConjunctionConstraint::testConstraintStateSpaceOrThrow(
    std::shared_ptr<TestableConstraint> constraint)
{
  if (constraint->getStateSpace() != mStateSpace) {
    throw std::invalid_argument{
        "Constriants must all be in specified StateSpace"};
  }
}
