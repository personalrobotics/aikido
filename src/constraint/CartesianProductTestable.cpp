#include <dart/common/StlHelpers.hpp>
#include <aikido/constraint/CartesianProductTestable.hpp>

namespace aikido {
namespace constraint {

using dart::common::make_unique;

//==============================================================================
CartesianProductTestable::CartesianProductTestable(
    std::shared_ptr<statespace::CartesianProduct> _stateSpace,
    std::vector<std::shared_ptr<Testable>> _constraints)
  : mStateSpace(std::move(_stateSpace)), mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("_stateSpace is nullptr.");

  for (std::size_t i = 0; i < mConstraints.size(); ++i)
  {
    if (!mConstraints[i])
    {
      std::stringstream msg;
      msg << i << "th constraint is null.";
      throw std::invalid_argument(msg.str());
    }
  }

  if (mConstraints.size() != mStateSpace->getNumSubspaces())
  {
    std::stringstream msg;
    msg << "Mismatch between size of CartesianProduct and the number of"
        << " constraints: " << mStateSpace->getNumSubspaces()
        << " != " << mConstraints.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  for (std::size_t i = 0; i < mStateSpace->getNumSubspaces(); ++i)
  {
    if (mConstraints[i]->getStateSpace() != mStateSpace->getSubspace<>(i))
    {
      std::stringstream msg;
      msg << "Constraint " << i << " is not defined over this StateSpace.";
      throw std::invalid_argument(msg.str());
    }
  }
}

//==============================================================================
statespace::StateSpacePtr CartesianProductTestable::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
bool CartesianProductTestable::isSatisfied(
    const aikido::statespace::StateSpace::State* _state,
    TestableOutcome* outcome) const
{
  auto defaultOutcomeObject = dynamic_cast_if_present<DefaultOutcome>(outcome);

  const auto state
      = static_cast<const statespace::CartesianProduct::State*>(_state);

  for (std::size_t i = 0; i < mStateSpace->getNumSubspaces(); ++i)
  {
    auto subState = mStateSpace->getSubState<>(state, i);
    if (!mConstraints[i]->isSatisfied(subState))
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
std::unique_ptr<TestableOutcome> CartesianProductTestable::createOutcome() const
{
  return std::unique_ptr<TestableOutcome>(new DefaultOutcome());
}

} // namespace constraint
} // namespace aikido
