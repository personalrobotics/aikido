#include <aikido/constraint/TestableIntersection.hpp>

#include <stdexcept>

namespace aikido {
namespace constraint {

//==============================================================================
TestableIntersection::TestableIntersection(
    statespace::ConstStateSpacePtr _stateSpace,
    std::vector<ConstTestablePtr> _constraints)
  : mStateSpace(std::move(_stateSpace)), mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("_statespace is nullptr.");

  for (auto c : mConstraints)
    testConstraintStateSpaceOrThrow(c);
}

//==============================================================================
bool TestableIntersection::isSatisfied(
    const aikido::statespace::StateSpace::State* _state,
    TestableOutcome* outcome) const
{
  auto defaultOutcomeObject
      = dynamic_cast_or_throw<DefaultTestableOutcome>(outcome);

  for (auto c : mConstraints)
  {
    if (!c->isSatisfied(_state))
    {
      if (defaultOutcomeObject)
        defaultOutcomeObject->setSatisfiedFlag(false);
      return false;
    }
  }

  if (defaultOutcomeObject)
    defaultOutcomeObject->setSatisfiedFlag(true);
  return true;
}

//==============================================================================
std::unique_ptr<TestableOutcome> TestableIntersection::createOutcome() const
{
  return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
}

//==============================================================================
statespace::ConstStateSpacePtr TestableIntersection::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
void TestableIntersection::addConstraint(ConstTestablePtr _constraint)
{
  if (_constraint->getStateSpace() == mStateSpace)
  {
    mConstraints.emplace_back(std::move(_constraint));
  }
  else
  {
    throw std::invalid_argument{
        "Constraints must all be in specified StateSpace"};
  }
}

//==============================================================================
void TestableIntersection::testConstraintStateSpaceOrThrow(
    const ConstTestablePtr& constraint)
{
  if (constraint->getStateSpace() != mStateSpace)
  {
    throw std::invalid_argument{
        "Constraints must all be in specified StateSpace"};
  }
}

} // namespace constraint
} // namespace aikido
