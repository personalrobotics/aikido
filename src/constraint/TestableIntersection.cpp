#include <aikido/constraint/TestableIntersection.hpp>

#include <stdexcept>

namespace aikido {
namespace constraint {

//==============================================================================
TestableIntersection::TestableIntersection(
    statespace::StateSpacePtr _stateSpace,
    std::vector<std::shared_ptr<Testable>> _constraints)
  : mStateSpace(std::move(_stateSpace)), mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("_statespace is nullptr.");

  for (auto c : mConstraints)
    testConstraintStateSpaceOrThrow(c);
}

//==============================================================================
bool TestableIntersection::isSatisfied(
    const aikido::statespace::StateSpace::State* _state, bool _givenState) const
{
  for (auto c : mConstraints)
  {
    if (!c->isSatisfied(_state, _givenState))
      return false;
  }
  return true;
}

//==============================================================================
statespace::StateSpacePtr TestableIntersection::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
void TestableIntersection::addConstraint(TestablePtr _constraint)
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
    const TestablePtr& constraint)
{
  if (constraint->getStateSpace() != mStateSpace)
  {
    throw std::invalid_argument{
        "Constraints must all be in specified StateSpace"};
  }
}

} // namespace constraint
} // namespace aikido
