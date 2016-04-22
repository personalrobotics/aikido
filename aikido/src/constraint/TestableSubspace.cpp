#include <dart/common/StlHelpers.h>
#include <aikido/constraint/TestableSubspace.hpp>

namespace aikido
{
namespace constraint
{
using dart::common::make_unique;

//=============================================================================
TestableSubspace::TestableSubspace(
  std::shared_ptr<statespace::CartesianProduct> _stateSpace,
  std::vector<std::shared_ptr<Testable>> _constraints)
: mStateSpace(std::move(_stateSpace))
, mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("_stateSpace is nullptr.");

  if (mConstraints.size() != mStateSpace->getNumStates()) {
    std::stringstream msg;
    msg << "Mismatch between size of CartesianProduct and the number of"
        << " constraints: " << mStateSpace->getNumStates()
        << " != " << mConstraints.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  for (size_t i = 0; i < mStateSpace->getNumStates(); ++i) {
    if (mConstraints[i]->getStateSpace() != mStateSpace->getSubSpace<>(i)) {
      std::stringstream msg;
      msg << "Constraint " << i << " is not defined over this StateSpace.";
      throw std::invalid_argument(msg.str());
    }
  }
}

//=============================================================================
statespace::StateSpacePtr TestableSubspace::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
bool TestableSubspace::isSatisfied(
    const aikido::statespace::StateSpace::State* _state) const
{
  const auto state
    = static_cast<const statespace::CartesianProduct::State*>(_state);

  for (size_t i = 0; i < mStateSpace->getNumStates(); ++i) {
    auto subState = mStateSpace->getSubState<>(state, i);
    if (!mConstraints[i]->isSatisfied(subState)) {
      return false;
    }
  }
  return true;
}

}  // namespace constraint
}  // namespace aikido
